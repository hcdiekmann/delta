import { hashSeed, Rng } from '../../math/rng';
import { vec3, type Vec3 } from '../../math/vec3';
import type { PresetName } from '../../kinematics/params';

export type FieldPresetId = 'raisedBeds' | 'flatField';

export interface FieldPreset {
  id: FieldPresetId;
  label: string;
  description: string;
  robot: PresetName;
  /** Centre lines (y) of all beds; the robot works on the bed at y = 0 */
  beds: number[];
  bedTopWidth: number;
  bedHeight: number;
  /** Horizontal width of each sloped bed side */
  bedSlope: number;
  /** Crop row offsets relative to the bed centre */
  rows: number[];
  cropSpacing: number;
  /** Half width of the strip the robot is responsible for */
  laneHalfWidth: number;
  /** Height of the robot base above the ground (bed top) */
  baseHeight: number;
}

export const FIELD_PRESETS: Record<FieldPresetId, FieldPreset> = {
  raisedBeds: {
    id: 'raisedBeds',
    label: 'Raised beds',
    description: 'Two crop rows per dam, compact weeder robot',
    robot: 'weeder',
    beds: [-1.35, 0, 1.35],
    bedTopWidth: 0.75,
    bedHeight: 0.2,
    bedSlope: 0.15,
    rows: [-0.19, 0.19],
    cropSpacing: 0.25,
    laneHalfWidth: 0.375,
    baseHeight: 0.74,
  },
  flatField: {
    id: 'flatField',
    label: 'Flat field',
    description: 'Four rows under a wide gantry, large robot',
    robot: 'picker',
    beds: [0],
    bedTopWidth: 6,
    bedHeight: 0,
    bedSlope: 0,
    rows: [-1.05, -0.75, -0.45, -0.15, 0.15, 0.45, 0.75, 1.05],
    cropSpacing: 0.2,
    laneHalfWidth: 0.6,
    baseHeight: 1.07,
  },
};

export type PlantKind = 'crop' | 'weed';

export interface Plant {
  id: number;
  kind: PlantKind;
  /** World position of the stem at ground level */
  pos: Vec3;
  /** Canopy radius and height [m] */
  radius: number;
  height: number;
  rotation: number;
  /** Visual variant for the renderer */
  variant: number;
  state: 'growing' | 'reserved' | 'carried' | 'binned' | 'missed' | 'damaged';
}

export const CHUNK_LENGTH = 4;

/** Ground height (top surface) at lateral position y for a preset. */
export function groundHeight(preset: FieldPreset, y: number): number {
  if (preset.bedHeight === 0) return 0;
  for (const c of preset.beds) {
    const d = Math.abs(y - c);
    const half = preset.bedTopWidth / 2;
    if (d <= half) return preset.bedHeight;
    if (d <= half + preset.bedSlope) return preset.bedHeight * (1 - (d - half) / preset.bedSlope);
  }
  return 0;
}

/**
 * Generate the plants of one field chunk deterministically from the seed and chunk index,
 * so the field is endless but every run with the same seed is identical.
 */
export function generateChunk(
  preset: FieldPreset,
  seed: number,
  index: number,
  weedDensity: number,
  nextId: () => number,
): Plant[] {
  const rng = new Rng(hashSeed(seed, index));
  const x0 = index * CHUNK_LENGTH;
  const plants: Plant[] = [];
  const top = preset.bedHeight;

  // Crops: regular spacing with a little jitter and some gaps (failed germination)
  for (const bed of preset.beds) {
    for (const row of preset.rows) {
      const y = bed + row;
      if (Math.abs(y - bed) > preset.bedTopWidth / 2) continue;
      for (let x = x0 + preset.cropSpacing / 2; x < x0 + CHUNK_LENGTH; x += preset.cropSpacing) {
        if (rng.next() < 0.05) continue;
        const radius = rng.range(0.05, 0.075);
        plants.push({
          id: nextId(),
          kind: 'crop',
          pos: vec3(x + rng.normal(0, 0.012), y + rng.normal(0, 0.01), top),
          radius,
          height: radius * rng.range(1.3, 1.7),
          rotation: rng.range(0, Math.PI * 2),
          variant: rng.int(0, 3),
          state: 'growing',
        });
      }
    }
  }

  // Weeds: random positions on the bed tops (Poisson count, rejection sampled around crops)
  for (const bed of preset.beds) {
    const half = Math.min(preset.bedTopWidth / 2, 1.2) - 0.03;
    const area = CHUNK_LENGTH * 2 * half;
    const expected = weedDensity * area;
    // Poisson sample via exponential inter-arrivals
    let count = 0;
    for (let acc = rng.exponential(1); acc < expected; acc += rng.exponential(1)) count++;
    for (let k = 0, tries = 0; k < count && tries < count * 20; tries++) {
      const pos = vec3(x0 + rng.range(0, CHUNK_LENGTH), bed + rng.range(-half, half), top);
      const tooClose = plants.some((p) => {
        const minD = p.kind === 'crop' ? p.radius * 0.5 + 0.02 : 0.04;
        return Math.hypot(p.pos.x - pos.x, p.pos.y - pos.y) < minD;
      });
      if (tooClose) continue;
      const radius = rng.range(0.018, 0.04);
      plants.push({
        id: nextId(),
        kind: 'weed',
        pos,
        radius,
        height: radius * rng.range(0.8, 1.6),
        rotation: rng.range(0, Math.PI * 2),
        variant: rng.int(0, 3),
        state: 'growing',
      });
      k++;
    }
  }
  return plants;
}
