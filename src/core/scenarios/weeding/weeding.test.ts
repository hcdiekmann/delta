import { describe, expect, it } from 'vitest';
import { runFor } from '../../sim/simLoop';
import { DEFAULT_WEEDING, WeedingScenario, type WeedingConfig } from './weeding';

function run(cfg: Partial<WeedingConfig>, seconds = 60) {
  const s = new WeedingScenario({ ...DEFAULT_WEEDING, ...cfg });
  runFor((dt) => s.step(dt), seconds);
  return s;
}

describe('WeedingScenario', () => {
  it('is deterministic for a seed', () => {
    expect(run({ seed: 3 }, 15).counters()).toEqual(run({ seed: 3 }, 15).counters());
  });

  it.each([
    ['raised beds, bin', { field: 'raisedBeds', disposal: 'bin' }],
    ['raised beds, furrow', { field: 'raisedBeds', disposal: 'furrow' }],
    ['flat field', { field: 'flatField' }],
  ] as [string, Partial<WeedingConfig>][])('%s: removes most weeds and never hits a crop', (_n, cfg) => {
    const s = run(cfg);
    const c = s.counters();
    console.log(_n, JSON.stringify(c));
    expect(c.cropHits).toBe(0);
    expect(c.removed).toBeGreaterThan(20);
    expect(c.removed / (c.removed + c.missed)).toBeGreaterThan(0.75);
  });

  it('stops after the configured run length', () => {
    const s = run({ runLength: 2, speed: 0.2 }, 15);
    expect(s.finished).toBe(true);
    expect(s.distance).toBeGreaterThanOrEqual(2);
    expect(s.distance).toBeLessThan(2.01);
  });
});
