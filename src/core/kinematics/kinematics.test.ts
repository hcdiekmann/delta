import { describe, expect, it } from 'vitest';
import fc from 'fast-check';
import { distance, vec3 } from '../math/vec3';
import { elbow, forwardKinematics, inverseKinematics, jointVelocities, type Joints } from './kinematics';
import { PRESETS, type DeltaParams } from './params';

const robots: [string, DeltaParams][] = Object.entries(PRESETS);
const deg = (d: number) => (d * Math.PI) / 180;

describe.each(robots)('kinematics (%s)', (_name, p) => {
  it('solves a point on the axis with three equal angles', () => {
    const r = inverseKinematics(p, vec3(0, 0, -0.8 * p.lowerArm - 0.3 * p.upperArm));
    expect(r.ok).toBe(true);
    if (!r.ok) return;
    expect(r.theta[1]).toBeCloseTo(r.theta[0], 12);
    expect(r.theta[2]).toBeCloseTo(r.theta[0], 12);
  });

  it('returns the knee-out solution (elbow outside the base radius)', () => {
    const r = inverseKinematics(p, vec3(0, 0, -p.lowerArm));
    expect(r.ok).toBe(true);
    if (!r.ok) return;
    expect(Math.cos(r.theta[0])).toBeGreaterThan(0);
  });

  it('FK(IK(p)) = p and IK(FK(theta)) = theta', () => {
    fc.assert(
      fc.property(
        fc.double({ min: deg(-80), max: deg(20), noNaN: true }),
        fc.double({ min: deg(-80), max: deg(20), noNaN: true }),
        fc.double({ min: deg(-80), max: deg(20), noNaN: true }),
        (a, b, c) => {
          const theta: Joints = [a, b, c];
          const pos = forwardKinematics(p, theta);
          if (!pos) return;
          const ik = inverseKinematics(p, pos, -1); // negative margin: ignore limits here
          expect(ik.ok).toBe(true);
          if (!ik.ok) return;
          for (let i = 0; i < 3; i++) expect(ik.theta[i]).toBeCloseTo(theta[i]!, 7);
        },
      ),
    );
  });

  it('keeps every rod at length L', () => {
    const pos = vec3(0.1, -0.15, -0.9 * p.lowerArm);
    const r = inverseKinematics(p, pos);
    expect(r.ok).toBe(true);
    if (!r.ok) return;
    for (const i of [0, 1, 2] as const) {
      const joint = vec3(
        pos.x + p.effectorRadius * Math.cos((i * 2 * Math.PI) / 3),
        pos.y + p.effectorRadius * Math.sin((i * 2 * Math.PI) / 3),
        pos.z,
      );
      expect(distance(joint, elbow(p, i, r.theta[i]))).toBeCloseTo(p.lowerArm, 10);
    }
  });

  it('handles horizontal upper arms without NaN', () => {
    const pos = forwardKinematics(p, [0, 0, 0]);
    expect(pos).not.toBeNull();
    const r = inverseKinematics(p, pos!);
    expect(r.ok).toBe(true);
    if (r.ok) r.theta.forEach((t) => expect(t).toBeCloseTo(0, 9));
  });

  it('reports unreachable targets instead of failing silently', () => {
    expect(inverseKinematics(p, vec3(0, 0, -5))).toMatchObject({ ok: false, reason: 'unreachable' });
    expect(inverseKinematics(p, vec3(3, 0, -0.5))).toMatchObject({ ok: false, reason: 'unreachable' });
  });

  it('reports joint limit violations', () => {
    const pos = forwardKinematics(p, [deg(10), deg(10), deg(10)]);
    expect(pos).not.toBeNull();
    expect(inverseKinematics({ ...p, thetaMax: 0 }, pos!)).toMatchObject({ ok: false, reason: 'jointLimit' });
  });

  it('FK returns null instead of throwing when the rods cannot meet', () => {
    expect(forwardKinematics({ ...p, lowerArm: 0.3 * p.upperArm }, [0, 0, 0])).toBeNull();
  });

  it('joint velocities match finite differences', () => {
    const pos = vec3(0.12, 0.05, -0.85 * p.lowerArm);
    const vel = vec3(0.3, -0.2, 0.1);
    const h = 1e-6;
    const a = inverseKinematics(p, pos);
    const b = inverseKinematics(p, vec3(pos.x + vel.x * h, pos.y + vel.y * h, pos.z + vel.z * h));
    expect(a.ok && b.ok).toBe(true);
    if (!a.ok || !b.ok) return;
    const analytic = jointVelocities(p, a.theta, pos, vel);
    for (let i = 0; i < 3; i++) expect(analytic[i]).toBeCloseTo((b.theta[i]! - a.theta[i]!) / h, 4);
  });
});

describe('joint limit convention', () => {
  it('matches the legacy machine: -50deg..+100deg measured downward', () => {
    const p = PRESETS.picker;
    const offset = Math.atan(27.5 / 347);
    expect(p.thetaMax).toBeCloseTo(deg(50) - offset, 12); // highest arm position, elbow above base
    expect(p.thetaMin).toBeCloseTo(-deg(100) - offset, 12); // lowest, elbow tucked under
    expect(elbow(p, 0, p.thetaMax).z).toBeGreaterThan(0);
    expect(elbow(p, 0, p.thetaMin).z).toBeLessThan(-0.9 * p.upperArm);
  });
});
