import { sim } from '@/state/simHandle';
import { fmt, cn } from '@/lib/utils';
import type { Phase } from '@/core/robot/controller';
import { usePoll } from './usePoll';

export function StatsHud() {
  usePoll(8);
  const s = sim.scenario;
  if (!s) return null;
  const metrics = s.metrics();
  return (
    <div className="pointer-events-auto glass w-[300px] rounded-2xl p-3">
      <div className="mb-2 flex items-center justify-between px-1">
        <span className="text-[11px] font-semibold tracking-[0.14em] text-slate-400 uppercase">
          Live metrics
        </span>
        <span className="tabular font-mono text-[11px] text-slate-500">t = {fmt(s.time, 1)} s</span>
      </div>
      <div className="grid grid-cols-2 gap-1.5">
        {metrics.map((m) => (
          <div key={m.label} className="rounded-lg bg-white/[0.04] px-2.5 py-1.5">
            <div className="truncate text-[11px] text-slate-400">{m.label}</div>
            <div
              className={cn(
                'tabular font-mono text-[15px] font-medium text-white',
                m.tone === 'bad' && 'text-bad',
                m.tone === 'good' && 'text-accent',
              )}
            >
              {fmt(m.value, m.digits ?? 0)}
              {m.unit && <span className="ml-0.5 text-[11px] font-normal text-slate-400">{m.unit}</span>}
            </div>
          </div>
        ))}
      </div>
      <PhaseBar phase={s.robot.phase} />
    </div>
  );
}

const PHASES: { id: Phase; label: string }[] = [
  { id: 'approach', label: 'Approach' },
  { id: 'grasp', label: 'Grasp' },
  { id: 'transfer', label: 'Transfer' },
  { id: 'release', label: 'Release' },
];

function PhaseBar({ phase }: { phase: Phase }) {
  return (
    <div className="mt-2 flex items-center gap-1 px-1">
      {PHASES.map((p) => (
        <div key={p.id} className="flex-1">
          <div
            className={cn('h-1 rounded-full bg-white/10 transition-colors', phase === p.id && 'bg-accent')}
          />
          <div
            className={cn('mt-1 text-center text-[10px] text-slate-500', phase === p.id && 'text-slate-200')}
          >
            {p.label}
          </div>
        </div>
      ))}
      <div
        className={cn(
          'ml-1 rounded px-1.5 py-0.5 text-[10px] text-slate-500',
          (phase === 'idle' || phase === 'return') && 'bg-white/10 text-slate-200',
        )}
      >
        {phase === 'return' ? 'Home' : 'Idle'}
      </div>
    </div>
  );
}
