import { dot, sub, vec3, type Vec3 } from '../math/vec3';
import { effectorJoint, elbow, elbowDerivative, type Joints } from './kinematics';
import { LEGS, type DeltaParams } from './params';

const G = 9.81;

/** Solve M^T x = f for a 3x3 matrix given by its rows (Cramer's rule). */
function solveTransposed(rows: [Vec3, Vec3, Vec3], f: Vec3): Vec3 | null {
  // columns of M^T are the rows of M
  const [a, b, c] = rows;
  const det = a.x * (b.y * c.z - b.z * c.y) - b.x * (a.y * c.z - a.z * c.y) + c.x * (a.y * b.z - a.z * b.y);
  if (Math.abs(det) < 1e-12) return null;
  const d = (x: Vec3, y: Vec3, z: Vec3) =>
    x.x * (y.y * z.z - y.z * z.y) - y.x * (x.y * z.z - x.z * z.y) + z.x * (x.y * y.z - x.z * y.y);
  return vec3(d(f, b, c) / det, d(a, f, c) / det, d(a, b, f) / det);
}

/**
 * Motor torque estimate with the common simplified delta model (Codourey): each lower arm's mass
 * is split half to the elbow and half to the effector, the upper arm is a rod rotating about the
 * motor axis. The effector force maps to the joints through the transposed Jacobian.
 *
 * Returns torques at the motor shaft (after the gearbox) in Nm.
 */
export function motorTorques(
  p: DeltaParams,
  theta: Joints,
  thetaAcc: Joints,
  pos: Vec3,
  acc: Vec3,
  payload = 0,
): Joints {
  const mEff = p.effectorMass + payload + 3 * p.lowerArmMass;
  const force = vec3(mEff * acc.x, mEff * acc.y, mEff * (acc.z + G));
  // Constraint rows: n_i . pdot = (n_i . E_i') thetadot_i  =>  J = A^-1 B,  tau = B A^-T F
  const n = LEGS.map((i) => sub(effectorJoint(p, i, pos), elbow(p, i, theta[i]))) as [Vec3, Vec3, Vec3];
  const lambda = solveTransposed(n, force);
  return LEGS.map((i) => {
    const b = dot(n[i], elbowDerivative(p, i, theta[i]));
    const tauEffector = lambda ? b * lambda[['x', 'y', 'z'][i] as 'x' | 'y' | 'z'] : 0;
    const mElbow = p.lowerArmMass; // half of two rods
    const inertia = (p.upperArmMass * p.upperArm ** 2) / 3 + mElbow * p.upperArm ** 2;
    const gravity = (p.upperArmMass / 2 + mElbow) * p.upperArm * G * Math.cos(theta[i]);
    const tauJoint = tauEffector + inertia * thetaAcc[i] + gravity;
    return tauJoint / p.gearRatio + p.motorInertia * thetaAcc[i] * p.gearRatio;
  }) as unknown as Joints;
}
