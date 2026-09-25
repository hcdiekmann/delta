import { RotateCcw } from 'lucide-react';
import { sim } from '@/state/simHandle';
import { useApp } from '@/state/store';
import { fmt } from '@/lib/utils';
import { usePoll } from './usePoll';

/** Shown when a weeding run with a fixed length has finished. */
export function RunSummary() {
  usePoll(4);
  const restart = useApp((s) => s.restart);
  const s = sim.scenario;
  if (!s || !s.finished) return null;
  return (
    <div className="absolute inset-0 grid place-items-center bg-black/30 backdrop-blur-[2px]">
      <div className="glass w-[380px] rounded-2xl p-5">
        <h2 className="text-lg font-semibold text-white">Run complete</h2>
        <p className="mb-4 text-sm text-slate-400">
          {fmt(s.time, 0)} s of simulated work. Change a setting and run again to compare.
        </p>
        <dl className="grid grid-cols-2 gap-2">
          {s.metrics().map((m) => (
            <div key={m.label} className="rounded-lg bg-white/5 px-3 py-2">
              <dt className="text-[11px] text-slate-400">{m.label}</dt>
              <dd className="tabular font-mono text-white">
                {fmt(m.value, m.digits ?? 0)}
                {m.unit && <span className="ml-0.5 text-xs text-slate-400">{m.unit}</span>}
              </dd>
            </div>
          ))}
        </dl>
        <button
          type="button"
          onClick={restart}
          className="mt-4 inline-flex w-full items-center justify-center gap-2 rounded-xl bg-accent px-4 py-2 text-sm font-semibold text-slate-900 hover:brightness-110"
        >
          <RotateCcw size={15} /> Run again
        </button>
      </div>
    </div>
  );
}
