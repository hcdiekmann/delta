import { describe, expect, it } from 'vitest';
import { runBenchmark } from './benchmark';
import { DEFAULT_PICK_PLACE } from './scenarios/pickPlace/pickPlace';

describe('runBenchmark', () => {
  it('returns stats for every strategy', () => {
    const r = runBenchmark({ kind: 'pickPlace', config: DEFAULT_PICK_PLACE, seeds: 2, seconds: 10 });
    expect(r.map((s) => s.strategy).sort()).toEqual(['edf', 'fifo', 'nearest']);
    for (const s of r) for (const m of s.metrics) expect(m.min).toBeLessThanOrEqual(m.mean + 1e-9);
  });
});
