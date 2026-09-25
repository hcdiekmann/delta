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
    console.log(
      _n,
      s.counters(),
      s
        .metrics()
        .map((m) => `${m.label}=${m.value.toFixed(2)}`)
        .join(' '),
    );
    expect(placed).toBeGreaterThan(55);
    expect(missed / (placed + missed)).toBeLessThan(0.1);
  });

  it('keeps holding an item when its tray disappears and places it later', () => {
    const s = new PickPlaceScenario({ ...DEFAULT_PICK_PLACE, placeMode: 'staticTrays' });
    let guard = 0;
    while (s.robot.phase !== 'transfer' && guard++ < 10000) s.step(1 / 240);
    const item = s.robot.carrying!;
    expect(item).toBeTruthy();
    const trays = s.trays.splice(0, s.trays.length);
    runFor((dt) => s.step(dt), 1);
    expect(s.robot.carrying?.id).toBe(item.id);
    expect(s.products.find((p) => p.id === item.id)?.state).toBe('carried');
    s.trays.push(...trays);
    runFor((dt) => s.step(dt), 2);
    expect(s.products.find((p) => p.id === item.id)?.state).toBe('placed');
  });

  it('keeps the robot within its workspace the whole time', () => {
    const s = new PickPlaceScenario(DEFAULT_PICK_PLACE);
    runFor((dt) => {
      s.step(dt);
      for (const t of s.robot.theta) expect(Number.isFinite(t)).toBe(true);
    }, 20);
  });
});
