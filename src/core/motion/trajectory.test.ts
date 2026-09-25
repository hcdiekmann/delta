import { describe, expect, it } from 'vitest';
import { distance, length, sub, vec3, type Vec3 } from '../math/vec3';
import { archMove, planIntercept, type ArchOptions, type Segment } from './trajectory';

const arch: ArchOptions = {
  travelZ: -0.7,
  minLift: 0.05,
  xy: { vMax: 3, aMax: 25, jMax: 500 },
  z: { vMax: 2, aMax: 25, jMax: 500 },
  overlap: 0.5,
};

/** Numerically check that velocity and acceleration are consistent with position (smoothness). */
function expectSmooth(seg: Segment) {
  const dt = 1e-4;
  for (let t = dt; t < seg.duration - dt; t += seg.duration / 97) {
    const a = seg.sample(t - dt);
    const b = seg.sample(t + dt);
    const m = seg.sample(t);
    const vNum = vec3((b.p.x - a.p.x) / (2 * dt), (b.p.y - a.p.y) / (2 * dt), (b.p.z - a.p.z) / (2 * dt));
    expect(length(sub(vNum, m.v))).toBeLessThan(1e-3);
    const aNum = vec3((b.v.x - a.v.x) / (2 * dt), (b.v.y - a.v.y) / (2 * dt), (b.v.z - a.v.z) / (2 * dt));
    expect(length(sub(aNum, m.a))).toBeLessThan(0.05 * (1 + length(m.a)));
  }
}

describe('archMove', () => {
  const from = vec3(-0.3, 0.1, -0.9);
  const to = vec3(0.4, -0.2, -0.85);
  const m = archMove(from, to, arch);

  it('starts and ends at rest at the endpoints', () => {
    expect(distance(m.sample(0).p, from)).toBeLessThan(1e-12);
    expect(distance(m.sample(m.duration).p, to)).toBeLessThan(1e-9);
    expect(length(m.sample(0).v)).toBeLessThan(1e-12);
    expect(length(m.sample(m.duration).v)).toBeLessThan(1e-9);
  });

  it('clears the travel height and never goes below the endpoints', () => {
    let maxZ = -Infinity;
    for (let t = 0; t <= m.duration; t += 0.001) {
      const z = m.sample(t).p.z;
      maxZ = Math.max(maxZ, z);
      expect(z).toBeGreaterThanOrEqual(Math.min(from.z, to.z) - 1e-9);
    }
    expect(maxZ).toBeCloseTo(arch.travelZ, 6);
  });

  it('is smooth', () => expectSmooth(m));
});

describe('planIntercept', () => {
  const cases: [string, Vec3, Vec3, Vec3][] = [
    ['from rest to a conveyor item', vec3(0, 0, -0.9), vec3(0, 0, 0), vec3(0.5, 0, 0)],
    ['from a moving item to a counter-flow tray', vec3(0.2, -0.3, -0.9), vec3(0.4, 0, 0), vec3(-0.3, 0, 0)],
    ['from moving back to rest', vec3(0.2, 0.3, -0.9), vec3(0.4, 0, 0), vec3(0, 0, 0)],
  ];

  it.each(cases)('%s: meets the target with matching velocity', (_n, startP, startV, targetV) => {
    const target = { p: vec3(-0.5, -0.3, -0.95), v: targetV };
    const plan = planIntercept({ p: startP, v: startV }, target, arch)!;
    expect(plan).not.toBeNull();
    const s0 = plan.sample(0);
    expect(distance(s0.p, startP)).toBeLessThan(1e-9);
    expect(distance(s0.v, vec3(startV.x, startV.y, 0))).toBeLessThan(1e-9);
    const end = plan.sample(plan.duration);
    const targetAtEnd = vec3(
      target.p.x + targetV.x * plan.duration,
      target.p.y + targetV.y * plan.duration,
      target.p.z,
    );
    expect(distance(end.p, targetAtEnd)).toBeLessThan(1e-6);
    expect(distance(end.v, targetV)).toBeLessThan(1e-6);
    expect(length(end.a)).toBeLessThan(1e-6);
    expectSmooth(plan);
  });

  it('returns null when the target cannot be caught in time', () => {
    const plan = planIntercept(
      { p: vec3(0, 0, -0.9), v: vec3() },
      { p: vec3(-1, 0, -0.9), v: vec3(-5, 0, 0) },
      arch,
      {
        maxTime: 1,
      },
    );
    expect(plan).toBeNull();
  });
});
