import { PickPlaceScenario, type PickPlaceConfig } from './scenarios/pickPlace/pickPlace';
import { WeedingScenario, type WeedingConfig } from './scenarios/weeding/weeding';
import { STRATEGIES, type Strategy } from './scheduling/scheduler';
import { runFor } from './sim/simLoop';

export type BenchmarkRequest =
  | { kind: 'pickPlace'; config: PickPlaceConfig; seeds: number; seconds: number }
  | { kind: 'weeding'; config: WeedingConfig; seeds: number; seconds: number };

export interface MetricStats {
  label: string;
  unit: string;
  higherIsBetter: boolean;
  mean: number;
  min: number;
  max: number;
}

export interface StrategyResult {
  strategy: Strategy;
  metrics: MetricStats[];
}

type Sample = { label: string; unit: string; higherIsBetter: boolean; value: number }[];

function runOne(req: BenchmarkRequest, strategy: Strategy, seed: number): Sample {
  if (req.kind === 'pickPlace') {
    const s = new PickPlaceScenario({ ...req.config, strategy, seed });
    runFor((dt) => s.step(dt), req.seconds);
    const c = s.counters();
    const cycles = s.robot.stats.cycleTimes;
    return [
      { label: 'Throughput', unit: 'picks/min', higherIsBetter: true, value: (c.placed / c.time) * 60 },
      {
        label: 'Pick rate',
        unit: '%',
        higherIsBetter: true,
        value: (100 * c.placed) / Math.max(1, c.placed + c.missed),
      },
      {
        label: 'Avg cycle',
        unit: 's',
        higherIsBetter: false,
        value: cycles.length ? cycles.reduce((a, b) => a + b, 0) / cycles.length : 0,
      },
    ];
  }
  const s = new WeedingScenario({ ...req.config, strategy, seed, runLength: 0 });
  runFor((dt) => s.step(dt), req.seconds);
  const c = s.counters();
  return [
    {
      label: 'Removal rate',
      unit: '%',
      higherIsBetter: true,
      value: (100 * c.removed) / Math.max(1, c.removed + c.missed),
    },
    { label: 'Weeds removed', unit: 'per min', higherIsBetter: true, value: (c.removed / c.time) * 60 },
    { label: 'Crop hits', unit: 'count', higherIsBetter: false, value: c.cropHits },
  ];
}

/**
 * Run every scheduling strategy on the same set of seeds with otherwise identical settings.
 * Runs are deterministic, so differences come from the strategy alone.
 */
export function runBenchmark(
  req: BenchmarkRequest,
  onProgress?: (done: number, total: number) => void,
): StrategyResult[] {
  const total = STRATEGIES.length * req.seeds;
  let done = 0;
  return STRATEGIES.map(({ id }) => {
    const samples: Sample[] = [];
    for (let k = 0; k < req.seeds; k++) {
      samples.push(runOne(req, id, 1000 + k * 7919));
      onProgress?.(++done, total);
    }
    const metrics = samples[0]!.map((m, j) => {
      const values = samples.map((s) => s[j]!.value);
      return {
        label: m.label,
        unit: m.unit,
        higherIsBetter: m.higherIsBetter,
        mean: values.reduce((a, b) => a + b, 0) / values.length,
        min: Math.min(...values),
        max: Math.max(...values),
      };
    });
    return { strategy: id, metrics };
  });
}
