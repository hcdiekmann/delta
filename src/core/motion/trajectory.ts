import { add, addScaled, lengthXY, scale, sub, vec3, ZERO, type Vec3 } from '../math/vec3';
import { doubleS, type Limits } from './doubleS';

/** Cartesian state of the end effector (plus the rotation of the 4th axis). */
export interface MotionState {
  p: Vec3;
  v: Vec3;
  a: Vec3;
  yaw: number;
}

export interface Segment {
  readonly duration: number;
  sample(t: number): MotionState;
}

export interface ArchOptions {
  /** Absolute height of the traverse. It is raised to at least max(start, end) + minLift. */
  travelZ: number;
  minLift: number;
  xy: Limits;
  z: Limits;
  /** 0..1: how much the horizontal traverse overlaps the vertical lift and descent. */
  overlap: number;
}

/** Quintic smoothstep ("smootherstep") and its derivatives: 0 -> 1 with zero velocity and acceleration at both ends. */
export const smoother = (x: number) => {
  const t = Math.min(1, Math.max(0, x));
  return {
    h: t * t * t * (t * (6 * t - 15) + 10),
    dh: 30 * t * t * (t - 1) * (t - 1),
    ddh: 60 * t * (t - 1) * (2 * t - 1),
  };
};

/**
 * Rest-to-rest "gate" move used by pick & place machines: lift, traverse, descend, where the
 * traverse starts before the lift is complete and ends after the descent has started. Each
 * axis follows a jerk-limited profile, so the path is smooth and acceleration is continuous.
 */
export function archMove(from: Vec3, to: Vec3, o: ArchOptions): Segment & { travelZ: number } {
  const zt = Math.max(o.travelZ, from.z + o.minLift, to.z + o.minLift);
  const up = doubleS(zt - from.z, o.z);
  const down = doubleS(zt - to.z, o.z);
  const dxy = vec3(to.x - from.x, to.y - from.y, 0);
  const dist = lengthXY(dxy);
  const dir = dist > 1e-12 ? scale(dxy, 1 / dist) : ZERO;
  const xy = doubleS(dist, o.xy);
  const tXY = up.duration * (1 - o.overlap);
  const tDown = Math.max(up.duration, tXY + xy.duration - o.overlap * down.duration);
  const duration = Math.max(tDown + down.duration, tXY + xy.duration);
  return {
    duration,
    travelZ: zt,
    sample(t: number): MotionState {
      const h = xy.sample(t - tXY);
      let z: number, vz: number, az: number;
      if (t < tDown) {
        const u = up.sample(t);
        z = from.z + u.s;
        vz = u.v;
        az = u.a;
      } else {
        const d = down.sample(t - tDown);
        z = zt - d.s;
        vz = -d.v;
        az = -d.a;
      }
      return {
        p: vec3(from.x + dir.x * h.s, from.y + dir.y * h.s, z),
        v: vec3(dir.x * h.v, dir.y * h.v, vz),
        a: vec3(dir.x * h.a, dir.y * h.a, az),
        yaw: 0,
      };
    },
  };
}

/** Duration of an arch move without building the full sampler (used when solving for intercepts). */
export const archDuration = (from: Vec3, to: Vec3, o: ArchOptions) => archMove(from, to, o).duration;

export interface MovingPoint {
  /** Position at t = 0 (planning time) */
  p: Vec3;
  /** Constant horizontal velocity */
  v: Vec3;
}

export interface InterceptPlan extends Segment {
  /** Where the target is predicted to be at the end of the move. */
  meet: Vec3;
}

/**
 * Plan a move that starts at `start` (moving with velocity start.v) and meets a target moving
 * with constant velocity, arriving with the same velocity as the target.
 *
 * The motion is split into a carrier and a relative move: p(t) = r(t) + c(t).
 *   c(t) = v_s (t - Phi(t)) + v_e Phi(t) blends the start velocity into the target velocity,
 *          with Phi' = smootherstep, so c'(0) = v_s, c'(T) = v_e and c'' = 0 at both ends.
 *   r(t) is a rest-to-rest arch move chosen so that p(T) equals the target position at T.
 * The duration T is the smallest T >= minTime with archDuration(T) <= T (scan + bisection).
 */
export function planIntercept(
  start: MovingPoint,
  target: MovingPoint,
  arch: ArchOptions,
  {
    minTime = 0,
    maxTime = 5,
    yawFrom = 0,
    yawTo = 0,
  }: { minTime?: number; maxTime?: number; yawFrom?: number; yawTo?: number } = {},
): InterceptPlan | null {
  const vs = vec3(start.v.x, start.v.y, 0);
  const ve = vec3(target.v.x, target.v.y, 0);
  const carrierEnd = (T: number) => scale(add(vs, ve), T / 2);
  const relEnd = (T: number) => sub(addScaled(target.p, ve, T), carrierEnd(T));
  const f = (T: number) => archDuration(start.p, relEnd(T), arch) - T;

  // Find the first T where the arch fits into the available time
  // Any T >= T* with f(T) <= 0 is valid (the relative move then simply finishes early),
  // minTime lets callers wait until a target has entered the workspace.
  const step = 0.02;
  let lo = Math.max(step, minTime);
  let hi = -1;
  if (f(lo) <= 0) hi = lo;
  for (let T = lo + step; hi < 0 && T <= maxTime; T += step) {
    if (f(T) <= 0) hi = T;
    else lo = T;
  }
  if (hi < 0) return null;
  if (hi !== lo) {
    for (let i = 0; i < 30; i++) {
      const mid = (lo + hi) / 2;
      if (f(mid) <= 0) hi = mid;
      else lo = mid;
    }
  }
  const T = hi;
  const rel = archMove(start.p, relEnd(T), arch);
  const meet = addScaled(target.p, ve, T);

  return {
    duration: T,
    meet,
    sample(t: number): MotionState {
      const tc = Math.min(Math.max(t, 0), T);
      const g = smoother(tc / T);
      // Phi(t) = T * (tau^6 - 3 tau^5 + 2.5 tau^4), Phi' = smootherstep, Phi'' = smootherstep' / T
      const tau = tc / T;
      const Phi = T * tau ** 4 * (tau * tau - 3 * tau + 2.5);
      const c = add(scale(vs, tc - Phi), scale(ve, Phi));
      const cv = add(scale(vs, 1 - g.h), scale(ve, g.h));
      const ca = scale(sub(ve, vs), g.dh / T);
      const r = rel.sample(tc);
      return {
        p: add(r.p, c),
        v: add(r.v, cv),
        a: add(r.a, ca),
        yaw: yawFrom + (yawTo - yawFrom) * g.h,
      };
    },
  };
}

/** Hold still (or keep moving at constant velocity) for a while. */
export function coast(from: MotionState, duration: number): Segment {
  return {
    duration,
    sample: (t) => ({ p: addScaled(from.p, from.v, t), v: from.v, a: ZERO, yaw: from.yaw }),
  };
}

/** Check a segment by sampling it; returns the first failing time or null if it is valid. */
export function validateSegment(
  seg: Segment,
  ok: (s: MotionState, t: number) => boolean,
  dt = 0.01,
): number | null {
  for (let t = 0; t < seg.duration; t += dt) if (!ok(seg.sample(t), t)) return t;
  return ok(seg.sample(seg.duration), seg.duration) ? null : seg.duration;
}
