import { useEffect, useRef, useState } from 'react';
import { Dialog } from 'radix-ui';
import { Play, Table2, BarChart3, X } from 'lucide-react';
import { useApp } from '@/state/store';
import type { BenchmarkRequest, StrategyResult } from '@/core/benchmark';
import { STRATEGIES } from '@/core/scheduling/scheduler';
import { SERIES_COLORS } from '@/lib/palette';
import { cn, fmt } from '@/lib/utils';
import { Segmented } from './components/controls';

type Status =
  | { state: 'idle' }
  | { state: 'running'; done: number; total: number }
  | { state: 'done'; results: StrategyResult[] };

const LABEL: Record<string, string> = { edf: 'Earliest deadline', fifo: 'FIFO', nearest: 'Nearest' };

/** Runs all scheduling strategies headless in a worker and compares them. */
export function Benchmark() {
  const open = useApp((s) => s.benchmarkOpen);
  const set = useApp((s) => s.set);
  const mode = useApp((s) => s.mode);
  const [seeds, setSeeds] = useState('5');
  const [seconds, setSeconds] = useState('60');
  const [status, setStatus] = useState<Status>({ state: 'idle' });
  const [view, setView] = useState<'chart' | 'table'>('chart');
  const worker = useRef<Worker | null>(null);

  useEffect(() => () => worker.current?.terminate(), []);

  const run = () => {
    const s = useApp.getState();
    const req: BenchmarkRequest =
      s.mode === 'weeding'
        ? { kind: 'weeding', config: s.weeding, seeds: Number(seeds), seconds: Number(seconds) }
        : { kind: 'pickPlace', config: s.pickPlace, seeds: Number(seeds), seconds: Number(seconds) };
    worker.current?.terminate();
    const w = new Worker(new URL('../workers/benchmark.worker.ts', import.meta.url), { type: 'module' });
    worker.current = w;
    setStatus({ state: 'running', done: 0, total: STRATEGIES.length * req.seeds });
    w.onmessage = (e) => {
      if (e.data.type === 'progress') setStatus({ state: 'running', done: e.data.done, total: e.data.total });
      else {
        setStatus({ state: 'done', results: e.data.results });
        w.terminate();
      }
    };
    w.postMessage(req);
  };

  return (
    <Dialog.Root open={open} onOpenChange={(o) => set('benchmarkOpen', o)}>
      <Dialog.Portal>
        <Dialog.Overlay className="fixed inset-0 bg-black/40 backdrop-blur-[2px]" />
        <Dialog.Content className="glass fixed top-1/2 left-1/2 max-h-[90vh] w-[640px] max-w-[calc(100vw-2rem)] -translate-x-1/2 -translate-y-1/2 overflow-y-auto rounded-2xl p-5 focus:outline-none">
          <div className="mb-1 flex items-start justify-between gap-4">
            <div>
              <Dialog.Title className="text-lg font-semibold text-white">Strategy benchmark</Dialog.Title>
              <Dialog.Description className="text-sm text-slate-400">
                Runs every scheduling strategy on the same random seeds with your current{' '}
                {mode === 'weeding' ? 'weeding' : 'pick & place'} settings. Simulations run in a background
                worker.
              </Dialog.Description>
            </div>
            <Dialog.Close
              className="rounded-lg p-1.5 text-slate-400 hover:bg-white/10 hover:text-white"
              aria-label="Close"
            >
              <X size={16} />
            </Dialog.Close>
          </div>

          <div className="my-4 grid grid-cols-[1fr_1fr_auto] items-end gap-3">
            <Segmented
              label="Seeds per strategy"
              value={seeds}
              onChange={setSeeds}
              options={['3', '5', '10'].map((v) => ({ value: v, label: v }))}
            />
            <Segmented
              label="Simulated time"
              value={seconds}
              onChange={setSeconds}
              options={[
                { value: '30', label: '30 s' },
                { value: '60', label: '60 s' },
                { value: '120', label: '2 min' },
              ]}
            />
            <button
              type="button"
              onClick={run}
              disabled={status.state === 'running'}
              className="inline-flex h-[34px] items-center gap-1.5 rounded-lg bg-accent px-3 text-sm font-semibold text-slate-900 hover:brightness-110 disabled:opacity-50"
            >
              <Play size={14} /> Run
            </button>
          </div>

          {status.state === 'running' && (
            <div className="space-y-1.5">
              <div className="h-1.5 overflow-hidden rounded-full bg-white/10">
                <div
                  className="h-full rounded-full bg-accent transition-all"
                  style={{ width: `${(100 * status.done) / status.total}%` }}
                />
              </div>
              <p className="tabular font-mono text-xs text-slate-400">
                {status.done} / {status.total} runs
              </p>
            </div>
          )}

          {status.state === 'done' && (
            <div className="space-y-3">
              <div className="flex items-center justify-between">
                <Legend />
                <div className="flex rounded-lg bg-white/5 p-0.5">
                  {(['chart', 'table'] as const).map((v) => (
                    <button
                      key={v}
                      type="button"
                      aria-label={v === 'chart' ? 'Chart view' : 'Table view'}
                      onClick={() => setView(v)}
                      className={cn(
                        'rounded-md p-1.5 text-slate-400 hover:text-white',
                        view === v && 'bg-white/10 text-white',
                      )}
                    >
                      {v === 'chart' ? <BarChart3 size={14} /> : <Table2 size={14} />}
                    </button>
                  ))}
                </div>
              </div>
              {view === 'chart' ? (
                <SmallMultiples results={status.results} />
              ) : (
                <ResultTable results={status.results} />
              )}
              <p className="text-[11px] text-slate-500">
                Bars show the mean over seeds, whiskers the range (min to max).
              </p>
            </div>
          )}
        </Dialog.Content>
      </Dialog.Portal>
    </Dialog.Root>
  );
}

function Legend() {
  return (
    <div className="flex gap-4 text-xs text-slate-300">
      {STRATEGIES.map((s, i) => (
        <span key={s.id} className="inline-flex items-center gap-1.5">
          <span className="size-2.5 rounded-sm" style={{ background: SERIES_COLORS[i] }} />
          {LABEL[s.id]}
        </span>
      ))}
    </div>
  );
}

/** One mini horizontal bar chart per metric (different units never share an axis). */
function SmallMultiples({ results }: { results: StrategyResult[] }) {
  const metrics = results[0]!.metrics;
  return (
    <div className="grid gap-3 sm:grid-cols-3">
      {metrics.map((m, j) => {
        const max = Math.max(...results.map((r) => r.metrics[j]!.max), 1e-9) * 1.02;
        const best = results.reduce((b, r) =>
          (
            m.higherIsBetter
              ? r.metrics[j]!.mean > b.metrics[j]!.mean
              : r.metrics[j]!.mean < b.metrics[j]!.mean
          )
            ? r
            : b,
        );
        return (
          <figure key={m.label} className="rounded-xl bg-white/[0.04] p-3">
            <figcaption className="mb-2 leading-tight">
              <div className="text-xs text-slate-300">
                {m.label} <span className="text-slate-500">[{m.unit}]</span>
              </div>
              <div className="text-[10px] text-slate-500">
                {m.higherIsBetter ? 'higher is better' : 'lower is better'}
              </div>
            </figcaption>
            <div className="space-y-2">
              {results.map((r, i) => {
                const s = r.metrics[j]!;
                const pct = (v: number) => `${(100 * v) / max}%`;
                return (
                  <div
                    key={r.strategy}
                    className="group grid grid-cols-[1fr_3.2rem] items-center gap-2"
                    title={`${LABEL[r.strategy]}: mean ${fmt(s.mean, 2)} ${m.unit} (min ${fmt(s.min, 2)}, max ${fmt(s.max, 2)})`}
                  >
                    <div className="relative h-5">
                      <div
                        className="absolute top-0.5 left-0 h-4 rounded-r-[4px] transition-opacity group-hover:opacity-80"
                        style={{ width: pct(s.mean), background: SERIES_COLORS[i] }}
                      />
                      {/* min-max whisker */}
                      <div
                        className="absolute top-1/2 h-px bg-slate-200/70"
                        style={{ left: pct(s.min), width: `calc(${pct(s.max)} - ${pct(s.min)})` }}
                      />
                      <div
                        className="absolute top-[5px] h-2.5 w-px bg-slate-200/70"
                        style={{ left: pct(s.min) }}
                      />
                      <div
                        className="absolute top-[5px] h-2.5 w-px bg-slate-200/70"
                        style={{ left: pct(s.max) }}
                      />
                    </div>
                    <span
                      className={cn(
                        'tabular text-right font-mono text-[11px] text-slate-300',
                        r === best && 'font-semibold text-white',
                      )}
                    >
                      {fmt(s.mean, s.mean >= 100 ? 0 : s.mean >= 10 ? 1 : 2)}
                    </span>
                  </div>
                );
              })}
            </div>
          </figure>
        );
      })}
    </div>
  );
}

function ResultTable({ results }: { results: StrategyResult[] }) {
  const metrics = results[0]!.metrics;
  return (
    <table className="w-full text-left text-xs">
      <thead className="text-slate-400">
        <tr>
          <th className="py-1.5 font-medium">Strategy</th>
          {metrics.map((m) => (
            <th key={m.label} className="py-1.5 text-right font-medium">
              {m.label} [{m.unit}]
            </th>
          ))}
        </tr>
      </thead>
      <tbody className="tabular font-mono text-slate-200">
        {results.map((r) => (
          <tr key={r.strategy} className="border-t border-line">
            <td className="py-1.5 font-sans">{LABEL[r.strategy]}</td>
            {r.metrics.map((s) => (
              <td key={s.label} className="py-1.5 text-right">
                {fmt(s.mean, 1)}{' '}
                <span className="text-slate-500">
                  ({fmt(s.min, 1)}–{fmt(s.max, 1)})
                </span>
              </td>
            ))}
          </tr>
        ))}
      </tbody>
    </table>
  );
}
