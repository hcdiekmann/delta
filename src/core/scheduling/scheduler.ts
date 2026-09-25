import { addScaled, lengthXY, sub, vec3, type Vec3 } from '../math/vec3';
import { isReachable } from '../kinematics/kinematics';
import type { DeltaParams } from '../kinematics/params';
import {
  planIntercept,
  validateSegment,
  type ArchOptions,
  type InterceptPlan,
  type MotionState,
} from '../motion/trajectory';

/** Something the robot can pick or place into, described in the robot frame. */
export interface Target {
  readonly id: number;
  /** Current grasp/release point in the robot frame */
  pos: Vec3;
  /** Current velocity in the robot frame (horizontal) */
  vel: Vec3;
  /** Orientation of the object around Z */
  yaw: number;
  /** Rotational symmetry period (PI for a box, 0 = any orientation works) */
  symmetry: number;
}

export type Strategy = 'fifo' | 'edf' | 'nearest';
export const STRATEGIES: { id: Strategy; label: string; hint: string }[] = [
  { id: 'edf', label: 'Earliest deadline', hint: 'Pick whatever leaves the reach zone first' },
  { id: 'fifo', label: 'First in, first out', hint: 'Pick the most downstream item first' },
  { id: 'nearest', label: 'Nearest', hint: 'Pick the closest item to the gripper' },
];

export interface PlanContext {
  params: DeltaParams;
  /** Tool length: planned positions are tool tips, the flange sits this far above [m] */
  tool: number;
  arch: ArchOptions;
  /** Joint limit margin used while planning [rad] */
  margin: number;
  graspTime: number;
  maxTime: number;
  /** Optional keep-out check, `dt` is the time from now */
  clear?: (p: Vec3, dt: number) => boolean;
}

/** Is a tool tip position reachable (the kinematics work on the flange above it)? */
export const tipReachable = (ctx: PlanContext, p: Vec3) =>
  isReachable(ctx.params, vec3(p.x, p.y, p.z + ctx.tool), ctx.margin);

/** Reach window of a target moving with constant velocity: [tIn, tOut] from now, or null. */
export function reachWindow(ctx: PlanContext, t: Target, horizon = 12, step = 0.05): [number, number] | null {
  const ok = (tau: number) => tipReachable(ctx, addScaled(t.pos, t.vel, tau));
  let tIn = -1;
  for (let tau = 0; tau <= horizon; tau += step) {
    if (ok(tau)) {
      tIn = tau;
      break;
    }
  }
  if (tIn < 0) return null;
  let tOut = tIn;
  while (tOut + step <= horizon && ok(tOut + step)) tOut += step;
  // refine the exit with bisection
  let lo = tOut;
  let hi = tOut + step;
  for (let i = 0; i < 12; i++) {
    const mid = (lo + hi) / 2;
    if (ok(mid)) lo = mid;
    else hi = mid;
  }
  return [tIn, lo];
}

/** Closest equivalent yaw to `current` for an object with the given symmetry period. */
export function nearestYaw(target: number, symmetry: number, current: number): number {
  if (symmetry <= 0) return current;
  return target + Math.round((current - target) / symmetry) * symmetry;
}

export function validatePlan(ctx: PlanContext, plan: InterceptPlan, startOffset: number): boolean {
  return (
    validateSegment(
      plan,
      (s: MotionState, t: number) =>
        tipReachable(ctx, s.p) && (ctx.clear ? ctx.clear(s.p, startOffset + t) : true),
      0.02,
    ) === null
  );
}

export interface Candidate {
  target: Target;
  window: [number, number];
}

export function orderCandidates(strategy: Strategy, cands: Candidate[], effector: Vec3): Candidate[] {
  const sorted = [...cands];
  if (strategy === 'edf') sorted.sort((a, b) => a.window[1] - b.window[1]);
  else if (strategy === 'fifo') {
    // most downstream = furthest along its direction of travel
    const along = (c: Candidate) => {
      const speed = lengthXY(c.target.vel);
      return speed > 1e-9 ? (c.target.pos.x * c.target.vel.x + c.target.pos.y * c.target.vel.y) / speed : 0;
    };
    sorted.sort((a, b) => along(b) - along(a));
  } else sorted.sort((a, b) => lengthXY(sub(a.target.pos, effector)) - lengthXY(sub(b.target.pos, effector)));
  return sorted;
}

/**
 * Plan an intercept for a target, waiting for it to enter the reach zone if needed.
 * `target` and `window` describe the target as of now; `offset` is the time from now at which the
 * move starts (target positions are extrapolated and keep-out checks are evaluated at that time).
 */
export function planFor(
  ctx: PlanContext,
  start: { p: Vec3; v: Vec3; yaw: number },
  target: Target,
  window: [number, number],
  offset: number,
  yawTo: number,
): InterceptPlan | null {
  const tp = addScaled(target.pos, target.vel, offset);
  const minTime = Math.max(0, window[0] - offset + 0.02);
  const maxTime = Math.min(ctx.maxTime, window[1] - offset - ctx.graspTime);
  if (maxTime <= minTime) return null;
  for (const speed of [1, 0.7]) {
    const arch: ArchOptions =
      speed === 1
        ? ctx.arch
        : {
            ...ctx.arch,
            xy: {
              vMax: ctx.arch.xy.vMax * speed,
              aMax: ctx.arch.xy.aMax * speed * speed,
              jMax: ctx.arch.xy.jMax * speed ** 3,
            },
          };
    const plan = planIntercept({ p: start.p, v: start.v }, { p: tp, v: target.vel }, arch, {
      minTime,
      maxTime,
      yawFrom: start.yaw,
      yawTo,
    });
    if (!plan) return null;
    if (validatePlan(ctx, plan, offset)) return plan;
  }
  return null;
}
