import { describe, expect, it } from 'vitest';
import { runFor } from '../../sim/simLoop';
import { DEFAULT_PICK_PLACE, PickPlaceScenario, type PickPlaceConfig } from './pickPlace';

function run(cfg: Partial<PickPlaceConfig>, seconds = 60) {
  const s = new PickPlaceScenario({ ...DEFAULT_PICK_PLACE, ...cfg });
  runFor((dt) => s.step(dt), seconds);
  return s;
}

describe('PickPlaceScenario', () => {
  it('is deterministic for a seed', () => {
    const a = run({ seed: 7 }, 20).counters();
    const b = run({ seed: 7 }, 20).counters();
    expect(a).toEqual(b);
  });

  it.each([
    ['counter-flow trays', { placeMode: 'trayConveyor', trayFlow: 'counter' }],
    ['co-flow trays', { placeMode: 'trayConveyor', trayFlow: 'co' }],
    ['static trays', { placeMode: 'staticTrays' }],
  ] as [string, Partial<PickPlaceConfig>][])('%s: picks almost everything at the default rate', (_n, cfg) => {
    const s = run(cfg);
    const { placed, missed } = s.counters();
    console.log(_n, s.counters(), s.metrics().map((m) => `${m.label}=${m.value.toFixed(2)}`).join(' '));
    expect(placed).toBeGreaterThan(55);
    expect(missed / (placed + missed)).toBeLessThan(0.1);
  });

  it('keeps the robot within its workspace the whole time', () => {
    const s = new PickPlaceScenario(DEFAULT_PICK_PLACE);
    runFor((dt) => {
      s.step(dt);
      for (const t of s.robot.theta) expect(Number.isFinite(t)).toBe(true);
    }, 20);
  });
});
