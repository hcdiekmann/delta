import { Rng } from '../../math/rng';
import { add, sub, vec3, type Vec3 } from '../../math/vec3';
import { PRESETS, type DeltaParams } from '../../kinematics/params';
import { RobotController, type TaskProvider } from '../../robot/controller';
import type { Strategy, Target } from '../../scheduling/scheduler';
import type { Metric, Scenario } from '../scenario';

export type PlaceMode = 'trayConveyor' | 'staticTrays';
export type TrayFlow = 'co' | 'counter';
export type ProductMix = 'boxes' | 'mixed';

export interface PickPlaceConfig {
  seed: number;
  strategy: Strategy;
  /** Product conveyor speed [m/s] */
  conveyorSpeed: number;
  /** Average product arrivals per second */
  spawnRate: number;
  placeMode: PlaceMode;
  trayFlow: TrayFlow;
  /** Tray conveyor speed [m/s] */
  traySpeed: number;
  /** Match the tray belt speed to the product rate so trays leave full */
  traySync: boolean;
  mix: ProductMix;
  /** Rotate products to a common orientation in the tray */
  alignProducts: boolean;
}

export const DEFAULT_PICK_PLACE: PickPlaceConfig = {
  seed: 42,
  strategy: 'edf',
  conveyorSpeed: 0.3,
  spawnRate: 1.2,
  placeMode: 'trayConveyor',
  trayFlow: 'counter',
  traySpeed: 0.15,
  traySync: true,
  mix: 'mixed',
  alignProducts: true,
};

/** Scene layout in world coordinates (Z up, metres). */
export const LAYOUT = {
  beltHeight: 0.8,
  baseHeight: 1.8,
  belt: { y: -0.27, width: 0.44, xStart: -2.4, xEnd: 2.4 },
  trayBelt: { y: 0.3, width: 0.38, xStart: -2.4, xEnd: 2.4 },
  tray: { size: 0.3, height: 0.04, pitch: 0.42, slots: 2 },
  staticTrays: [vec3(-0.25, 0.32, 0), vec3(0.15, 0.32, 0)],
  /** Items further downstream than this (relative to the robot) count as missed */
  missX: 0.9,
} as const;

export type ProductKind = 'box' | 'cylinder';
export const PRODUCT_SIZE: Record<ProductKind, { x: number; y: number; h: number }> = {
  box: { x: 0.12, y: 0.08, h: 0.05 },
  cylinder: { x: 0.08, y: 0.08, h: 0.06 },
};

export interface Product {
  id: number;
  kind: ProductKind;
  color: number;
  /** World position of the bottom centre */
  pos: Vec3;
  yaw: number;
  state: 'belt' | 'reserved' | 'carried' | 'placed' | 'missed';
  trayId?: number;
}

export interface Tray {
  id: number;
  /** World position of the tray centre (bottom) */
  pos: Vec3;
  slots: (number | null)[];
  static: boolean;
}

const SLOT_ID_BASE = 1_000_000;

export class PickPlaceScenario implements Scenario, TaskProvider {
  readonly id = 'pickPlace' as const;
  readonly params: DeltaParams = PRESETS.picker;
  readonly robot: RobotController;
  readonly products: Product[] = [];
  readonly trays: Tray[] = [];
  time = 0;
  readonly finished = false;

  private rng: Rng;
  private nextId = 1;
  private nextSpawn = 0;
  private nextTrayId = 1;
  private placed = 0;
  private missed = 0;
  private traysDone = 0;
  private traysIncomplete = 0;
  /** Distance the conveyors moved, for animating the belt texture */
  beltTravel = 0;
  trayTravel = 0;

  constructor(public config: PickPlaceConfig = DEFAULT_PICK_PLACE) {
    this.rng = new Rng(config.seed);
    const pickZ = -(LAYOUT.baseHeight - LAYOUT.beltHeight) + PRODUCT_SIZE.box.h;
    this.robot = new RobotController(
      {
        params: this.params,
        arch: {
          travelZ: pickZ + 0.07,
          minLift: 0.04,
          // about 6 g peak acceleration, typical for light delta pickers
          xy: { vMax: 5, aMax: 60, jMax: 3000 },
          z: { vMax: 3, aMax: 60, jMax: 3000 },
          overlap: 0.7,
        },
        strategy: config.strategy,
        graspTime: 0.08,
        releaseTime: 0.06,
        home: vec3(0, 0.05, pickZ + 0.1),
        margin: 0.05,
        maxInterceptTime: 3,
      },
      this,
    );
    this.setupTrays();
    // pre-fill the belt so the demo starts busy
    for (let t = 0; t < 6; t += 1 / 30) this.advanceWorld(1 / 30, false);
  }

  /** Update settings that can change while running. */
  configure(patch: Partial<PickPlaceConfig>) {
    this.config = { ...this.config, ...patch };
    this.robot.config.strategy = this.config.strategy;
  }

  robotBase(): Vec3 {
    return vec3(0, 0, LAYOUT.baseHeight);
  }

  /** Effective tray belt speed [m/s] */
  get traySpeed(): number {
    if (!this.config.traySync) return this.config.traySpeed;
    const slots = LAYOUT.tray.slots ** 2;
    return (this.config.spawnRate * LAYOUT.tray.pitch * 1.05) / slots;
  }

  private get trayVelocity(): number {
    if (this.config.placeMode === 'staticTrays') return 0;
    return this.config.trayFlow === 'co' ? this.traySpeed : -this.traySpeed;
  }

  private newTray(pos: Vec3, isStatic: boolean): Tray {
    const n = LAYOUT.tray.slots * LAYOUT.tray.slots;
    return { id: this.nextTrayId++, pos, slots: Array(n).fill(null), static: isStatic };
  }

  private setupTrays() {
    if (this.config.placeMode === 'staticTrays') {
      for (const p of LAYOUT.staticTrays) this.trays.push(this.newTray(vec3(p.x, p.y, LAYOUT.beltHeight), true));
      return;
    }
    const { xStart, xEnd, y } = LAYOUT.trayBelt;
    for (let x = xStart + 0.2; x < xEnd; x += LAYOUT.tray.pitch)
      this.trays.push(this.newTray(vec3(x, y, LAYOUT.beltHeight), false));
  }

  // ---- world simulation ---------------------------------------------------------------------

  private spawnProduct() {
    const kind: ProductKind = this.config.mix === 'boxes' || this.rng.next() < 0.6 ? 'box' : 'cylinder';
    const halfSpan = LAYOUT.belt.width / 2 - 0.07;
    const y = LAYOUT.belt.y + this.rng.range(-halfSpan, halfSpan);
    const pos = vec3(LAYOUT.belt.xStart, y, LAYOUT.beltHeight);
    // avoid overlapping with the previous products at the belt entry
    const blocked = this.products.some(
      (p) => p.state === 'belt' && Math.hypot(p.pos.x - pos.x, p.pos.y - pos.y) < 0.15,
    );
    if (blocked) return false;
    const palette = [0xf97316, 0x38bdf8, 0xa3e635, 0xf43f5e, 0xfacc15];
    this.products.push({
      id: this.nextId++,
      kind,
      color: this.rng.pick(palette),
      pos,
      yaw: this.rng.range(-Math.PI, Math.PI),
      state: 'belt',
    });
    return true;
  }

  private advanceWorld(dt: number, withRobot: boolean) {
    const v = this.config.conveyorSpeed;
    const tv = this.trayVelocity;
    this.beltTravel += v * dt;
    this.trayTravel += tv * dt;

    this.nextSpawn -= dt;
    if (this.nextSpawn <= 0) {
      if (this.spawnProduct()) this.nextSpawn = this.rng.exponential(this.config.spawnRate);
      else this.nextSpawn = 0.05;
    }

    for (const p of this.products) {
      if (p.state === 'belt' || p.state === 'reserved' || p.state === 'missed')
        p.pos = vec3(p.pos.x + v * dt, p.pos.y, p.pos.z);
      if (p.state === 'belt' && p.pos.x > LAYOUT.missX && withRobot) {
        p.state = 'missed';
        this.missed++;
      }
    }

    for (const tray of this.trays) {
      if (tray.static) continue;
      tray.pos = vec3(tray.pos.x + tv * dt, tray.pos.y, tray.pos.z);
    }
    // products in trays move with them
    for (const p of this.products) {
      if (p.state !== 'placed') continue;
      const tray = this.trays.find((t) => t.id === p.trayId);
      if (tray && !tray.static) p.pos = vec3(p.pos.x + tv * dt, p.pos.y, p.pos.z);
    }

    this.recycleTrays(withRobot);
    // remove products that fell off the end of the belt
    for (let i = this.products.length - 1; i >= 0; i--) {
      const p = this.products[i]!;
      if ((p.state === 'belt' || p.state === 'missed') && p.pos.x > LAYOUT.belt.xEnd) {
        if (p.state === 'belt' && withRobot) this.missed++;
        this.products.splice(i, 1);
      }
    }
  }

  private recycleTrays(count: boolean) {
    const { xStart, xEnd } = LAYOUT.trayBelt;
    for (let i = this.trays.length - 1; i >= 0; i--) {
      const tray = this.trays[i]!;
      const full = tray.slots.every((s) => s !== null);
      // an operator swaps full static trays instantly, moving trays leave at the belt end
      const leave = tray.static ? full : tray.pos.x > xEnd + 0.2 || tray.pos.x < xStart - 0.2;
      if (!leave) continue;
      if (count) {
        if (full) this.traysDone++;
        else if (tray.slots.some((s) => s !== null)) this.traysIncomplete++;
      }
      for (let k = this.products.length - 1; k >= 0; k--)
        if (this.products[k]!.trayId === tray.id) this.products.splice(k, 1);
      this.trays.splice(i, 1);
      if (tray.static) this.trays.push(this.newTray(tray.pos, true));
    }
    if (this.config.placeMode === 'trayConveyor') {
      // keep the tray belt filled at a fixed pitch
      const tv = this.trayVelocity;
      const upstream = tv >= 0 ? xStart : xEnd;
      const nearest = this.trays.reduce((m, t) => Math.min(m, Math.abs(t.pos.x - upstream)), Infinity);
      if (nearest >= LAYOUT.tray.pitch) {
        const x = tv >= 0 ? upstream + nearest - LAYOUT.tray.pitch : upstream - nearest + LAYOUT.tray.pitch;
        this.trays.push(this.newTray(vec3(x, LAYOUT.trayBelt.y, LAYOUT.beltHeight), false));
      }
    }
  }

  step(dt: number) {
    this.time += dt;
    this.advanceWorld(dt, true);
    this.robot.step(this.time, dt);
    // carried products follow the gripper
    const carried = this.robot.carrying;
    if (carried) {
      const p = this.products.find((q) => q.id === carried.id);
      if (p) {
        const eff = add(this.robotBase(), this.robot.state.p);
        p.pos = vec3(eff.x, eff.y, eff.z - PRODUCT_SIZE[p.kind].h);
        p.yaw = this.robot.state.yaw - this.robot.carryYawOffset;
      }
    }
  }

  // ---- task provider ------------------------------------------------------------------------

  private toRobot(world: Vec3): Vec3 {
    return sub(world, this.robotBase());
  }

  private productTarget(p: Product): Target {
    return {
      id: p.id,
      pos: this.toRobot(vec3(p.pos.x, p.pos.y, p.pos.z + PRODUCT_SIZE[p.kind].h)),
      vel: vec3(this.config.conveyorSpeed, 0, 0),
      yaw: p.yaw,
      symmetry: p.kind === 'box' ? Math.PI : 0,
    };
  }

  /** Slot centre in world coordinates. */
  slotPosition(tray: Tray, k: number): Vec3 {
    const n = LAYOUT.tray.slots;
    const pitch = LAYOUT.tray.size / n;
    const ix = k % n;
    const iy = Math.floor(k / n);
    return vec3(
      tray.pos.x + (ix - (n - 1) / 2) * pitch,
      tray.pos.y + (iy - (n - 1) / 2) * pitch,
      tray.pos.z + LAYOUT.tray.height,
    );
  }

  private slotTarget(tray: Tray, k: number, itemHeight: number): Target {
    const s = this.slotPosition(tray, k);
    return {
      id: SLOT_ID_BASE + tray.id * 16 + k,
      pos: this.toRobot(vec3(s.x, s.y, s.z + itemHeight + 0.005)),
      vel: vec3(tray.static ? 0 : this.trayVelocity, 0, 0),
      yaw: this.config.alignProducts ? 0 : Number.NaN,
      symmetry: Math.PI,
    };
  }

  pickCandidates(): Target[] {
    return this.products.filter((p) => p.state === 'belt').map((p) => this.productTarget(p));
  }

  placeCandidates(item: Target): Target[] {
    const product = this.products.find((p) => p.id === item.id);
    const h = product ? PRODUCT_SIZE[product.kind].h : 0.05;
    const tv = this.trayVelocity;
    // fill the most downstream tray first so trays leave full
    const trays = [...this.trays].sort((a, b) => (tv >= 0 ? b.pos.x - a.pos.x : a.pos.x - b.pos.x));
    const out: Target[] = [];
    for (const tray of trays) {
      // skip trays that have already passed the robot
      if (tv > 0 && tray.pos.x > LAYOUT.missX) continue;
      if (tv < 0 && tray.pos.x < -LAYOUT.missX) continue;
      tray.slots.forEach((s, k) => {
        if (s === null) {
          const t = this.slotTarget(tray, k, h);
          // keep the product orientation if alignment is off
          out.push(Number.isNaN(t.yaw) ? { ...t, yaw: item.yaw } : t);
        }
      });
      if (out.length >= 8) break;
    }
    return out;
  }

  find(id: number): Target | undefined {
    if (id >= SLOT_ID_BASE) {
      const trayId = Math.floor((id - SLOT_ID_BASE) / 16);
      const k = (id - SLOT_ID_BASE) % 16;
      const tray = this.trays.find((t) => t.id === trayId);
      if (!tray) return undefined;
      const carried = this.robot.carrying && this.products.find((p) => p.id === this.robot.carrying!.id);
      const t = this.slotTarget(tray, k, carried ? PRODUCT_SIZE[carried.kind].h : 0.05);
      return t;
    }
    const p = this.products.find((q) => q.id === id);
    if (!p || p.state === 'missed' || p.state === 'placed') return undefined;
    return this.productTarget(p);
  }

  onPickScheduled(item: Target) {
    const p = this.products.find((q) => q.id === item.id);
    if (p) p.state = 'reserved';
  }

  onGrasp(item: Target) {
    const p = this.products.find((q) => q.id === item.id);
    if (p) p.state = 'carried';
  }

  onRelease(item: Target, place: Target) {
    const p = this.products.find((q) => q.id === item.id);
    const trayId = Math.floor((place.id - SLOT_ID_BASE) / 16);
    const k = (place.id - SLOT_ID_BASE) % 16;
    const tray = this.trays.find((t) => t.id === trayId);
    if (!p || !tray) return;
    tray.slots[k] = p.id;
    p.state = 'placed';
    p.trayId = tray.id;
    const s = this.slotPosition(tray, k);
    p.pos = s;
    this.placed++;
  }

  onAbort(item: Target) {
    const p = this.products.find((q) => q.id === item.id);
    if (!p) return;
    if (p.state === 'carried') {
      // dropped: count as missed
      p.state = 'missed';
      this.missed++;
    } else if (p.state === 'reserved') p.state = 'belt';
  }

  payloadMass(): number {
    return 0.3;
  }

  // ---- metrics ------------------------------------------------------------------------------

  metrics(): Metric[] {
    const s = this.robot.stats;
    const minutes = Math.max(this.time, 1) / 60;
    const avgCycle = s.cycleTimes.length ? s.cycleTimes.reduce((a, b) => a + b, 0) / s.cycleTimes.length : 0;
    const total = this.placed + this.missed;
    return [
      { label: 'Picks / min', value: this.placed / minutes, digits: 1 },
      { label: 'Placed', value: this.placed },
      { label: 'Missed', value: this.missed, tone: this.missed > 0 ? 'bad' : 'neutral' },
      { label: 'Pick rate', value: total ? (100 * this.placed) / total : 100, unit: '%', digits: 1, tone: 'good' },
      { label: 'Avg cycle', value: avgCycle, unit: 's', digits: 2 },
      { label: 'Utilisation', value: (100 * s.busyTime) / Math.max(s.totalTime, 1e-9), unit: '%', digits: 0 },
      { label: 'Trays filled', value: this.traysDone },
      ...(this.traysIncomplete ? [{ label: 'Trays incomplete', value: this.traysIncomplete, tone: 'bad' as const }] : []),
    ];
  }

  /** Raw counters for tests and the benchmark. */
  counters() {
    return { placed: this.placed, missed: this.missed, time: this.time };
  }
}
