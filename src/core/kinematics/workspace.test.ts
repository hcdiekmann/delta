import { describe, expect, it } from 'vitest';
import { vec3 } from '../math/vec3';
import { isReachable } from './kinematics';
import { PRESETS } from './params';
import { bestWorkingHeight, computeWorkspace } from './workspace';

describe.each(Object.entries(PRESETS))('workspace (%s)', (_name, p) => {
  const ws = computeWorkspace(p, { slices: 12, azimuths: 36 })!;

  it('has a sensible vertical extent', () => {
    expect(ws).not.toBeNull();
    expect(ws.zTop).toBeLessThan(0);
    expect(ws.zBottom).toBeLessThan(ws.zTop - 0.2);
  });

  it('boundary is reachable just inside and not just outside', () => {
    for (const s of ws.slices.slice(1, -1)) {
      s.radii.forEach((r, k) => {
        if (r < 0.02) return;
        const phi = (k / s.radii.length) * 2 * Math.PI;
        const at = (rr: number) => vec3(rr * Math.cos(phi), rr * Math.sin(phi), s.z);
        expect(isReachable(p, at(r - 1e-4))).toBe(true);
        expect(isReachable(p, at(r + 1e-3))).toBe(false);
      });
    }
  });

  it('has 120 degree symmetry', () => {
    const s = ws.slices[Math.floor(ws.slices.length / 2)]!;
    const n = s.radii.length;
    for (let k = 0; k < n; k++) expect(s.radii[(k + n / 3) % n]).toBeCloseTo(s.radii[k]!, 6);
  });

  it('finds a working height with a usable reach', () => {
    const z = bestWorkingHeight(ws);
    expect(z).toBeLessThan(ws.zTop);
    expect(z).toBeGreaterThan(ws.zBottom);
  });
});
