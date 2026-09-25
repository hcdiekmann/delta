import { Rng } from '../../math/rng';
import { add, sub, vec3, type Vec3 } from '../../math/vec3';
import { PRESETS, type DeltaParams } from '../../kinematics/params';
import { RobotController, type TaskProvider } from '../../robot/controller';
import type { Strategy, Target } from '../../scheduling/scheduler';
import type { Metric, Scenario } from '../scenario';
import { CHUNK_LENGTH, FIELD_PRESETS, generateChunk, type FieldPreset, type FieldPresetId, type Plant } from './field';

export type ClearancePolicy = 'skip' | 'attempt';
export type Disposal = 'bin' | 'furrow';

export interface WeedingConfig {
  seed: number;
  field: FieldPresetId;
  strategy: Strategy;
  /** Vehicle speed [m/s] */
  speed: number;
  /** Weeds per square metre */
  weedDensity: number;
  /** Classifier threshold: detections with a weed confidence above this are treated as weeds */
  threshold: number;
  /** Extra distance to keep between the gripper and crops [m] */
  clearance: number;
  clearancePolicy: ClearancePolicy;
  disposal: Disposal;
  /** Stop after this many metres (0 = endless) */
  runLength: number;
}

export const DEFAULT_WEEDING: WeedingConfig = {
  seed: 42,
  field: 'raisedBeds',
  strategy: 'edf',
  speed: 0.12,
  weedDensity: 5,
  threshold: 0.5,
  clearance: 0.01,
  clearancePolicy: 'skip',
  disposal: 'bin',
  runLength: 0,
};

/** Camera field of view on the ground, relative to the robot base (x ahead) */
export const VISION = { xNear: 0.55, xFar: 0.95, noise: 0.006 };
const GRIPPER_RADIUS = 0.022;
const GRIP_HEIGHT = 0.012;

export interface Detection {
  plantId: number;
  /** Measured (noisy) world position */
  pos: Vec3;
  /** Classifier confidence that this is a weed */
  confidence: number;
  label: 'weed' | 'crop';
  /** Weed too close to a crop to be removed safely */
  tooClose: boolean;
  time: number;
}

export class WeedingScenario implements Scenario, TaskProvider {
  readonly id = 'weeding' as const;
  readonly params: DeltaParams;
  readonly preset: FieldPreset;
  readonly robot: RobotController;
  readonly plants = new Map<number, Plant>();
  readonly detections = new Map<number, Detection>();
  time = 0;
  /** Vehicle position along the field [m] and its current speed */
  vehicleX = -1.5;
  vehicleSpeed = 0;
  finished = false;

  private rng: Rng;
  private chunks = new Map<number, number[]>();
  private nextPlantId = 1;
  private startX: number;
  private counts = {
    removed: 0,
    missed: 0,
    cropHits: 0,
    cropsPulled: 0,
    skipped: 0,
    truePositive: 0,
    falsePositive: 0,
    falseNegative: 0,
  };
  private hitCrops = new Set<number>();
  /** Number of weeds in the bin (emptied when full) */
  binFill = 0;
  static readonly BIN_CAPACITY = 60;
  private furrowSide = 1;

  constructor(public config: WeedingConfig = DEFAULT_WEEDING) {
    this.preset = FIELD_PRESETS[config.field];
    this.params = PRESETS[this.preset.robot];
    this.rng = new Rng(config.seed ^ 0x5eed);
    this.startX = this.vehicleX;
    const groundZ = -this.preset.baseHeight;
    const travelZ = groundZ + 0.16;
    this.robot = new RobotController(
      {
        params: this.params,
        arch: {
          travelZ,
          minLift: 0.04,
          xy: { vMax: 3, aMax: 40, jMax: 2000 },
          z: { vMax: 2, aMax: 40, jMax: 2000 },
          // lift and descend vertically so the gripper never sweeps sideways through the canopy
          overlap: 0,
        },
        strategy: config.strategy,
        graspTime: 0.12,
        releaseTime: 0.08,
        home: vec3(0.1, 0, travelZ + 0.02),
        margin: 0.05,
        maxInterceptTime: 3,
      },
      this,
    );
    this.vehicleSpeed = config.speed;
    this.streamChunks();
    // the vehicle drives in from behind: everything between the robot and the camera is already seen
    this.detect(this.vehicleX - 1);
  }

  configure(patch: Partial<WeedingConfig>) {
    this.config = { ...this.config, ...patch };
    this.robot.config.strategy = this.config.strategy;
  }

  robotBase(): Vec3 {
    return vec3(this.vehicleX, 0, this.preset.bedHeight + this.preset.baseHeight);
  }

  get distance() {
    return this.vehicleX - this.startX;
  }

  private streamChunks() {
    const first = Math.floor((this.vehicleX - 4) / CHUNK_LENGTH);
    const last = Math.floor((this.vehicleX + 8) / CHUNK_LENGTH);
    for (let i = first; i <= last; i++) {
      if (this.chunks.has(i)) continue;
      const plants = generateChunk(this.preset, this.config.seed, i, this.config.weedDensity, () => this.nextPlantId++);
      for (const p of plants) this.plants.set(p.id, p);
      this.chunks.set(
        i,
        plants.map((p) => p.id),
      );
    }
    for (const [i, ids] of this.chunks) {
      if (i >= first) continue;
      for (const id of ids) {
        this.plants.delete(id);
        this.detections.delete(id);
      }
      this.chunks.delete(i);
    }
  }

  // ---- geometry helpers -----------------------------------------------------------------------

  private toRobot(world: Vec3): Vec3 {
    return sub(world, this.robotBase());
  }

  private inLane(p: Plant) {
    return Math.abs(p.pos.y) <= this.preset.laneHalfWidth;
  }

  private nearestCropDistance(pos: Vec3): number {
    let best = Infinity;
    for (const p of this.plants.values()) {
      if (p.kind !== 'crop' || Math.abs(p.pos.x - pos.x) > 0.2) continue;
      best = Math.min(best, Math.hypot(p.pos.x - pos.x, p.pos.y - pos.y) - p.radius);
    }
    return best;
  }

  // ---- simulation -------------------------------------------------------------------------------

  step(dt: number) {
    if (this.finished) return;
    this.time += dt;
    // vehicle follows the set speed with limited acceleration
    const dv = this.config.speed - this.vehicleSpeed;
    this.vehicleSpeed += Math.sign(dv) * Math.min(Math.abs(dv), 0.3 * dt);
    this.vehicleX += this.vehicleSpeed * dt;
    this.streamChunks();
    this.detect();
    this.robot.step(this.time, dt);
    this.updateCarried();
    this.checkCollisions();
    this.checkMissed();
    if (this.config.runLength > 0 && this.distance >= this.config.runLength) this.finished = true;
  }

  /** Simulated camera: classify plants entering the field of view. */
  private detect(x0 = this.vehicleX + VISION.xNear) {
    const x1 = this.vehicleX + VISION.xFar;
    for (const p of this.plants.values()) {
      if (this.detections.has(p.id) || p.pos.x < x0 || p.pos.x > x1 || !this.inLane(p)) continue;
      const confidence = p.kind === 'weed' ? this.rng.beta(9, 1.6) : this.rng.beta(1.2, 14);
      const label = confidence >= this.config.threshold ? 'weed' : 'crop';
      const pos = vec3(p.pos.x + this.rng.normal(0, VISION.noise), p.pos.y + this.rng.normal(0, VISION.noise), p.pos.z);
      const tooClose =
        label === 'weed' &&
        this.nearestCropDistance(pos) < GRIPPER_RADIUS + this.config.clearance &&
        this.config.clearancePolicy === 'skip';
      this.detections.set(p.id, { plantId: p.id, pos, confidence, label, tooClose, time: this.time });
      if (p.kind === 'weed' && label === 'weed') this.counts.truePositive++;
      if (p.kind === 'crop' && label === 'weed') this.counts.falsePositive++;
      if (p.kind === 'weed' && label === 'crop') this.counts.falseNegative++;
      if (tooClose) this.counts.skipped++;
    }
  }

  private updateCarried() {
    const carried = this.robot.carrying;
    if (!carried) return;
    const p = this.plants.get(carried.id);
    if (!p) return;
    const eff = add(this.robotBase(), this.robot.state.p);
    p.pos = vec3(eff.x, eff.y, eff.z - GRIP_HEIGHT);
  }

  /** Ground truth collision check between the gripper and crops, independent of the planner. */
  private checkCollisions() {
    const eff = add(this.robotBase(), this.robot.state.p);
    for (const p of this.plants.values()) {
      if (p.kind !== 'crop' || p.state !== 'growing' || this.hitCrops.has(p.id)) continue;
      if (Math.abs(p.pos.x - eff.x) > 0.15) continue;
      const d = Math.hypot(p.pos.x - eff.x, p.pos.y - eff.y);
      if (d < p.radius * 0.7 && eff.z < p.pos.z + p.height * 0.8) {
        this.hitCrops.add(p.id);
        this.counts.cropHits++;
      }
    }
  }

  private checkMissed() {
    const behind = this.vehicleX - 0.7;
    for (const p of this.plants.values()) {
      if (p.kind !== 'weed' || p.state !== 'growing' || p.pos.x > behind || p.pos.x < behind - 0.1) continue;
      if (!this.inLane(p)) continue;
      p.state = 'missed';
      // weeds deliberately left because they grow too close to a crop are counted separately
      if (!this.detections.get(p.id)?.tooClose) this.counts.missed++;
    }
  }

  // ---- task provider ----------------------------------------------------------------------------

  private plantTarget(p: Plant, d: Detection): Target {
    return {
      id: p.id,
      pos: this.toRobot(vec3(d.pos.x, d.pos.y, d.pos.z + GRIP_HEIGHT)),
      vel: vec3(-this.vehicleSpeed, 0, 0),
      yaw: 0,
      symmetry: 0,
    };
  }

  pickCandidates(): Target[] {
    const out: Target[] = [];
    for (const d of this.detections.values()) {
      if (d.label !== 'weed' || d.tooClose) continue;
      const p = this.plants.get(d.plantId);
      if (p && p.state === 'growing') out.push(this.plantTarget(p, d));
    }
    return out;
  }

  private static readonly BIN_ID = -1;
  private static readonly FURROW_ID = -2;

  /** Drop point above the bin, which hangs next to the robot over the furrow (robot frame). */
  binPosition(): Vec3 {
    return this.preset.robot === 'weeder' ? vec3(-0.05, 0.4, -0.5) : vec3(-0.2, 0.6, -0.78);
  }

  placeCandidates(): Target[] {
    if (this.config.disposal === 'bin')
      return [{ id: WeedingScenario.BIN_ID, pos: this.binPosition(), vel: vec3(), yaw: 0, symmetry: 0 }];
    // drop between the rows / into the furrow, moving with the ground
    const y = this.preset.robot === 'weeder' ? 0.44 : 0.66;
    const z = this.robot.config.arch.travelZ;
    return [
      {
        id: WeedingScenario.FURROW_ID,
        pos: vec3(-0.05, this.furrowSide * y, z),
        vel: vec3(-this.vehicleSpeed, 0, 0),
        yaw: 0,
        symmetry: 0,
      },
    ];
  }

  find(id: number): Target | undefined {
    if (id < 0) return this.placeCandidates()[0];
    const p = this.plants.get(id);
    const d = this.detections.get(id);
    if (!p || !d || (p.state !== 'growing' && p.state !== 'reserved')) return undefined;
    return this.plantTarget(p, d);
  }

  onPickScheduled(item: Target) {
    const p = this.plants.get(item.id);
    if (p) p.state = 'reserved';
    this.furrowSide = item.pos.y >= 0 ? 1 : -1;
  }

  onGrasp(item: Target) {
    const p = this.plants.get(item.id);
    if (!p) return;
    // the gripper closes on whatever is at the measured spot: a misclassified crop gets pulled too
    p.state = 'carried';
    if (p.kind === 'crop') this.counts.cropsPulled++;
  }

  onRelease(item: Target) {
    const p = this.plants.get(item.id);
    if (!p) return;
    if (p.kind === 'weed') this.counts.removed++;
    if (this.config.disposal === 'bin') {
      p.state = 'binned';
      this.binFill = (this.binFill + 1) % WeedingScenario.BIN_CAPACITY;
    } else {
      // dropped into the furrow where it dries out
      p.state = 'damaged';
      p.pos = vec3(p.pos.x, p.pos.y, 0);
    }
  }

  onAbort(item: Target) {
    const p = this.plants.get(item.id);
    if (p && (p.state === 'reserved' || p.state === 'carried')) p.state = 'growing';
  }

  /** Keep the gripper out of the crop canopies (using detected crops, moving with the field). */
  clear(pos: Vec3, dt: number): boolean {
    const base = this.robotBase();
    const shift = this.vehicleSpeed * dt;
    const wx = pos.x + base.x + shift;
    const wy = pos.y + base.y;
    const wz = pos.z + base.z;
    for (const d of this.detections.values()) {
      if (d.label !== 'crop') continue;
      const p = this.plants.get(d.plantId);
      if (!p || p.kind !== 'crop' || Math.abs(d.pos.x - wx) > 0.2) continue;
      const keepOut = this.config.clearancePolicy === 'skip' ? p.radius + GRIPPER_RADIUS : 0.015 + GRIPPER_RADIUS;
      if (wz < d.pos.z + p.height + 0.02 && Math.hypot(d.pos.x - wx, d.pos.y - wy) < keepOut) return false;
    }
    return true;
  }

  payloadMass(): number {
    return 0.02;
  }

  // ---- metrics ------------------------------------------------------------------------------------

  metrics(): Metric[] {
    const c = this.counts;
    const minutes = Math.max(this.time, 1) / 60;
    const handled = c.removed + c.missed;
    return [
      { label: 'Weeds removed', value: c.removed, tone: 'good' },
      { label: 'Weeds / min', value: c.removed / minutes, digits: 1 },
      { label: 'Removal rate', value: handled ? (100 * c.removed) / handled : 100, unit: '%', digits: 1 },
      { label: 'Missed', value: c.missed, tone: c.missed ? 'bad' : 'neutral' },
      { label: 'Crop hits', value: c.cropHits, tone: c.cropHits ? 'bad' : 'good' },
      { label: 'Crops pulled', value: c.cropsPulled, tone: c.cropsPulled ? 'bad' : 'good' },
      { label: 'Skipped (near crop)', value: c.skipped },
      { label: 'Detections TP / FP / FN', value: c.truePositive, unit: ` / ${c.falsePositive} / ${c.falseNegative}` },
      { label: 'Distance', value: this.distance, unit: 'm', digits: 1 },
      {
        label: 'Utilisation',
        value: (100 * this.robot.stats.busyTime) / Math.max(this.robot.stats.totalTime, 1e-9),
        unit: '%',
        digits: 0,
      },
    ];
  }

  counters() {
    return { ...this.counts, time: this.time, distance: this.distance };
  }
}
