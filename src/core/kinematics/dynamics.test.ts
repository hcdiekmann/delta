import { describe, expect, it } from 'vitest';
import { vec3 } from '../math/vec3';
import { inverseKinematics, jointVelocities } from './kinematics';
import { motorTorques } from './dynamics';
import { PRESETS } from './params';

describe('motorTorques', () => {
  const p = PRESETS.picker;

  it('is symmetric on the axis and positive (holding against gravity)', () => {
    const pos = vec3(0, 0, -0.9);
    const ik = inverseKinematics(p, pos);
    if (!ik.ok) throw new Error('unreachable');
    const tau = motorTorques(p, ik.theta, [0, 0, 0], pos, vec3());
    expect(tau[1]).toBeCloseTo(tau[0], 9);
    expect(tau[2]).toBeCloseTo(tau[0], 9);
    expect(tau[0]).toBeGreaterThan(0);
  });

  it('effector part obeys virtual work: sum(tau_i * thetadot_i) = F . v', () => {
    const pos = vec3(0.15, -0.1, -0.95);
    const vel = vec3(0.4, 0.2, -0.3);
    const ik = inverseKinematics(p, pos);
    if (!ik.ok) throw new Error('unreachable');
    const td = jointVelocities(p, ik.theta, pos, vel);
    // massless arms isolate the effector term
    const q = { ...p, upperArmMass: 0, lowerArmMass: 0, motorInertia: 0, gearRatio: 1 };
    const acc = vec3(3, -2, 5);
    const tau = motorTorques(q, ik.theta, [0, 0, 0], pos, acc);
    const power = tau[0] * td[0] + tau[1] * td[1] + tau[2] * td[2];
    const F = vec3(q.effectorMass * acc.x, q.effectorMass * acc.y, q.effectorMass * (acc.z + 9.81));
    expect(power).toBeCloseTo(F.x * vel.x + F.y * vel.y + F.z * vel.z, 9);
  });
});
