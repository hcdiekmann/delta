import { vec3 } from '../math/vec3';
import { isReachable } from './kinematics';
import type { DeltaParams } from './params';

export interface WorkspaceSlice {
  z: number;
  /** Reachable radius per azimuth (azimuth k = k * 2PI / radii.length) */
  radii: number[];
}

export interface Workspace {
  zTop: number;
  zBottom: number;
  slices: WorkspaceSlice[];
}

function bisect(ok: (x: number) => boolean, good: number, bad: number, iterations = 30): number {
  for (let i = 0; i < iterations; i++) {
    const mid = (good + bad) / 2;
    if (ok(mid)) good = mid;
    else bad = mid;
  }
  return good;
}

/** Largest reachable radius in direction phi at height z, assuming the slice is star-shaped. */
export function reachRadius(p: DeltaParams, z: number, phi: number, margin = 0): number {
  const c = Math.cos(phi);
  const s = Math.sin(phi);
  const ok = (r: number) => isReachable(p, vec3(r * c, r * s, z), margin);
  if (!ok(0)) return 0;
  const step = 0.01;
  const max = p.upperArm + p.lowerArm + p.baseRadius;
  let r = 0;
  while (r + step < max && ok(r + step)) r += step;
  return bisect(ok, r, r + step);
}

/** Vertical extent of the workspace along the central axis. */
export function axisRange(p: DeltaParams, margin = 0): { zTop: number; zBottom: number } | null {
  const ok = (z: number) => isReachable(p, vec3(0, 0, z), margin);
  const depth = p.upperArm + p.lowerArm;
  const step = 0.005;
  let first: number | null = null;
  let last: number | null = null;
  for (let z = 0; z > -depth; z -= step) {
    if (ok(z)) {
      if (first === null) first = z;
      last = z;
    } else if (first !== null) break;
  }
  if (first === null || last === null) return null;
  return { zTop: bisect(ok, first, first + step), zBottom: bisect(ok, last, last - step) };
}

/**
 * Sample the reachable workspace as horizontal slices with radial bisection.
 * The delta workspace is star-shaped around its axis in each slice, which keeps this cheap
 * (about slices * azimuths * 30 IK evaluations).
 */
export function computeWorkspace(
  p: DeltaParams,
  { slices = 32, azimuths = 72, margin = 0 }: { slices?: number; azimuths?: number; margin?: number } = {},
): Workspace | null {
  const range = axisRange(p, margin);
  if (!range) return null;
  const { zTop, zBottom } = range;
  const out: WorkspaceSlice[] = [];
  for (let k = 0; k < slices; k++) {
    // slightly inset top and bottom so the end slices are not degenerate
    const z = zTop + (zBottom - zTop) * (0.002 + (0.996 * k) / (slices - 1));
    const radii: number[] = [];
    for (let a = 0; a < azimuths; a++) radii.push(reachRadius(p, z, (a / azimuths) * 2 * Math.PI, margin));
    out.push({ z, radii });
  }
  return { zTop, zBottom, slices: out };
}

/** Middle of the height band where the horizontal reach is (nearly) largest: a good working plane. */
export function bestWorkingHeight(ws: Workspace): number {
  const minR = ws.slices.map((s) => Math.min(...s.radii));
  const best = Math.max(...minR);
  const band = ws.slices.filter((_, i) => minR[i]! >= 0.98 * best);
  return (band[0]!.z + band[band.length - 1]!.z) / 2;
}
