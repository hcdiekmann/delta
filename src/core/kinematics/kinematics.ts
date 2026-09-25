import { add, addScaled, cross, dot, lengthSq, scale, sub, vec3, type Vec3 } from '../math/vec3';
import { LEG_ANGLES, LEGS, type DeltaParams, type LegIndex } from './params';

export type Joints = readonly [number, number, number];

export type IkFailure = 'unreachable' | 'jointLimit' | 'ballJoint';
export type IkResult = { ok: true; theta: Joints } | { ok: false; reason: IkFailure; leg: LegIndex };

/** Unit vector pointing radially outward for leg i (in the base plane). */
export const legRadial = (i: LegIndex): Vec3 => vec3(Math.cos(LEG_ANGLES[i]), Math.sin(LEG_ANGLES[i]), 0);
/** Motor axis direction for leg i (tangential). */
export const legAxis = (i: LegIndex): Vec3 => vec3(-Math.sin(LEG_ANGLES[i]), Math.cos(LEG_ANGLES[i]), 0);

/** Motor axis position of leg i. */
export const baseJoint = (p: DeltaParams, i: LegIndex): Vec3 => scale(legRadial(i), p.baseRadius);

/** Elbow position of leg i for joint angle theta. */
export function elbow(p: DeltaParams, i: LegIndex, theta: number): Vec3 {
  const r = legRadial(i);
  return vec3(
    r.x * (p.baseRadius + p.upperArm * Math.cos(theta)),
    r.y * (p.baseRadius + p.upperArm * Math.cos(theta)),
    p.upperArm * Math.sin(theta),
  );
}

/** Derivative of the elbow position with respect to theta. */
export function elbowDerivative(p: DeltaParams, i: LegIndex, theta: number): Vec3 {
  const r = legRadial(i);
  return vec3(
    -r.x * p.upperArm * Math.sin(theta),
    -r.y * p.upperArm * Math.sin(theta),
    p.upperArm * Math.cos(theta),
  );
}

/** Lower arm attachment point on the effector for leg i. */
export const effectorJoint = (p: DeltaParams, i: LegIndex, pos: Vec3): Vec3 =>
  addScaled(pos, legRadial(i), p.effectorRadius);

/** Solve the angle of a single leg without limit checks, NaN if unreachable. */
function solveLeg(p: DeltaParams, i: LegIndex, pos: Vec3): number {
  // rotate the target into the leg frame (leg along +x)
  const c = Math.cos(-LEG_ANGLES[i]);
  const s = Math.sin(-LEG_ANGLES[i]);
  const a = c * pos.x - s * pos.y + p.effectorRadius - p.baseRadius;
  const b = s * pos.x + c * pos.y;
  const z = pos.z;
  // |P - E(theta)|^2 = L^2  <=>  A cos(theta) + B sin(theta) = C
  const A = 2 * a * p.upperArm;
  const B = 2 * z * p.upperArm;
  const C = a * a + b * b + z * z + p.upperArm * p.upperArm - p.lowerArm * p.lowerArm;
  const R = Math.hypot(A, B);
  if (R < 1e-12) return Number.NaN;
  const ratio = C / R;
  if (ratio > 1 + 1e-12 || ratio < -1 - 1e-12) return Number.NaN;
  // "+" branch = elbow outward (knee-out), the physical configuration
  return Math.atan2(B, A) + Math.acos(Math.min(1, Math.max(-1, ratio)));
}

/** Normalise an angle into (-PI, PI]. */
const wrap = (a: number) => Math.atan2(Math.sin(a), Math.cos(a));

/** Out-of-plane angle of the lower arm of leg i (for the ball joint limit). */
export function lowerArmTilt(p: DeltaParams, i: LegIndex, theta: number, pos: Vec3): number {
  const rod = sub(effectorJoint(p, i, pos), elbow(p, i, theta));
  return Math.asin(Math.min(1, Math.abs(dot(rod, legAxis(i))) / p.lowerArm));
}

/**
 * Inverse kinematics: effector position -> joint angles.
 * Returns why a position is not reachable instead of failing silently.
 * `margin` shrinks the joint and ball-joint limits (rad), useful for planning with some headroom.
 */
export function inverseKinematics(p: DeltaParams, pos: Vec3, margin = 0): IkResult {
  const theta: number[] = [];
  for (const i of LEGS) {
    const t = solveLeg(p, i, pos);
    if (Number.isNaN(t)) return { ok: false, reason: 'unreachable', leg: i };
    const w = wrap(t);
    if (w < p.thetaMin + margin || w > p.thetaMax - margin)
      return { ok: false, reason: 'jointLimit', leg: i };
    if (lowerArmTilt(p, i, w, pos) > p.ballJointMax - margin)
      return { ok: false, reason: 'ballJoint', leg: i };
    theta.push(w);
  }
  return { ok: true, theta: theta as unknown as Joints };
}

/** True if the position can be reached within limits (with optional margin). */
export const isReachable = (p: DeltaParams, pos: Vec3, margin = 0): boolean =>
  inverseKinematics(p, pos, margin).ok;

/**
 * Forward kinematics: joint angles -> effector position.
 * Intersects three spheres of radius L around the elbows (shifted by the effector offset)
 * and returns the lower solution, or null if the spheres do not intersect.
 */
export function forwardKinematics(p: DeltaParams, theta: Joints): Vec3 | null {
  const c = LEGS.map((i) => sub(elbow(p, i, theta[i]), scale(legRadial(i), p.effectorRadius)));
  const [c0, c1, c2] = c as [Vec3, Vec3, Vec3];
  // Subtracting the sphere equations gives two planes n.x = d; their intersection is a line
  const n1 = sub(c1, c0);
  const n2 = sub(c2, c0);
  const d1 = (lengthSq(c1) - lengthSq(c0)) / 2;
  const d2 = (lengthSq(c2) - lengthSq(c0)) / 2;
  const u = cross(n1, n2);
  const uu = lengthSq(u);
  if (uu < 1e-18) return null;
  const p0 = scale(add(scale(cross(n2, u), d1), scale(cross(u, n1), d2)), 1 / uu);
  // |p0 + t u - c0|^2 = L^2
  const w = sub(p0, c0);
  const qa = uu;
  const qb = 2 * dot(u, w);
  const qc = lengthSq(w) - p.lowerArm * p.lowerArm;
  const disc = qb * qb - 4 * qa * qc;
  if (disc < 0) return null;
  const sq = Math.sqrt(disc);
  const s1 = addScaled(p0, u, (-qb + sq) / (2 * qa));
  const s2 = addScaled(p0, u, (-qb - sq) / (2 * qa));
  return s1.z < s2.z ? s1 : s2;
}

/**
 * Joint velocities for a given effector velocity (inverse Jacobian).
 * From d/dt |P_i - E_i(theta_i)|^2 = 0:  thetaDot_i = (n_i . v) / (n_i . E_i'(theta_i)).
 */
export function jointVelocities(p: DeltaParams, theta: Joints, pos: Vec3, vel: Vec3): Joints {
  return LEGS.map((i) => {
    const n = sub(effectorJoint(p, i, pos), elbow(p, i, theta[i]));
    const den = dot(n, elbowDerivative(p, i, theta[i]));
    return Math.abs(den) < 1e-12 ? 0 : dot(n, vel) / den;
  }) as unknown as Joints;
}

/**
 * Conditioning measure in [0, 1]: 1 = well conditioned, near 0 = close to a singularity.
 * Combines the inverse singularity (rod aligned with the upper arm motion) and the forward
 * singularity (rods coplanar).
 */
export function conditioning(p: DeltaParams, theta: Joints, pos: Vec3): number {
  let inv = 1;
  const rods: Vec3[] = [];
  for (const i of LEGS) {
    const n = sub(effectorJoint(p, i, pos), elbow(p, i, theta[i]));
    rods.push(scale(n, 1 / p.lowerArm));
    const tangent = scale(elbowDerivative(p, i, theta[i]), 1 / p.upperArm);
    inv = Math.min(inv, Math.abs(dot(rods[i]!, tangent)));
  }
  const fwd = Math.abs(dot(rods[0]!, cross(rods[1]!, rods[2]!)));
  return Math.min(inv, fwd);
}

/** All joint positions needed to draw the robot. */
export interface RobotPose {
  theta: Joints;
  effector: Vec3;
  base: [Vec3, Vec3, Vec3];
  elbows: [Vec3, Vec3, Vec3];
  effectorJoints: [Vec3, Vec3, Vec3];
}

export function poseFromJoints(p: DeltaParams, theta: Joints, effector: Vec3): RobotPose {
  return {
    theta,
    effector,
    base: LEGS.map((i) => baseJoint(p, i)) as RobotPose['base'],
    elbows: LEGS.map((i) => elbow(p, i, theta[i])) as RobotPose['elbows'],
    effectorJoints: LEGS.map((i) => effectorJoint(p, i, effector)) as RobotPose['effectorJoints'],
  };
}

/** A sensible default working position: arms 30deg below horizontal. */
export function homePosition(p: DeltaParams): Vec3 {
  const t = -Math.PI / 6;
  return forwardKinematics(p, [t, t, t]) ?? vec3(0, 0, -p.lowerArm);
}
