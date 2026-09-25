import { describe, expect, it } from 'vitest';
import fc from 'fast-check';
import { doubleS, type Limits } from './doubleS';

const limits: Limits = { vMax: 3, aMax: 30, jMax: 600 };

describe('doubleS', () => {
  it('handles zero distance', () => {
    const p = doubleS(0, limits);
    expect(p.duration).toBe(0);
    expect(p.sample(1).s).toBe(0);
  });

  it('reaches the endpoints exactly, respects limits and is continuous', () => {
    fc.assert(
      fc.property(
        fc.double({ min: 1e-4, max: 5, noNaN: true }),
        fc.double({ min: 0.2, max: 5, noNaN: true }),
        fc.double({ min: 2, max: 60, noNaN: true }),
        fc.double({ min: 50, max: 2000, noNaN: true }),
        (D, vMax, aMax, jMax) => {
          const p = doubleS(D, { vMax, aMax, jMax });
          expect(p.sample(0).s).toBe(0);
          expect(p.sample(p.duration).s).toBeCloseTo(D, 9);
          const n = 400;
          const dt = p.duration / n;
          let prev = p.sample(0);
          for (let k = 1; k <= n; k++) {
            const cur = p.sample(k * dt);
            expect(cur.v).toBeLessThanOrEqual(vMax * (1 + 1e-9));
            expect(Math.abs(cur.a)).toBeLessThanOrEqual(aMax * (1 + 1e-9));
            expect(cur.v).toBeGreaterThanOrEqual(-1e-9);
            // continuity: position and velocity change consistently with bounded derivatives
            expect(Math.abs(cur.s - prev.s)).toBeLessThanOrEqual(vMax * dt * (1 + 1e-6) + 1e-12);
            expect(Math.abs(cur.v - prev.v)).toBeLessThanOrEqual(aMax * dt * (1 + 1e-6) + 1e-12);
            expect(Math.abs(cur.a - prev.a)).toBeLessThanOrEqual(jMax * dt * (1 + 1e-6) + 1e-9);
            prev = cur;
          }
        },
      ),
      { numRuns: 200 },
    );
  });

  it('velocity integrates to position', () => {
    const p = doubleS(1.3, limits);
    const n = 20000;
    const dt = p.duration / n;
    let s = 0;
    for (let k = 0; k < n; k++) s += p.sample((k + 0.5) * dt).v * dt;
    expect(s).toBeCloseTo(1.3, 5);
  });
});
