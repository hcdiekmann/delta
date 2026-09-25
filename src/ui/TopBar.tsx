import { Dices, Pause, Play, RotateCcw, SlidersHorizontal, StepForward, BarChart3 } from 'lucide-react';
import { useApp, type Mode } from '@/state/store';
import { sim } from '@/state/simHandle';
import { cn } from '@/lib/utils';
import { IconButton, Tip } from './components/controls';

const MODES: { id: Mode; label: string; hint: string }[] = [
  {
    id: 'pickPlace',
    label: 'Pick & Place',
    hint: 'Pick products from a moving conveyor and place them into trays',
  },
  { id: 'weeding', label: 'Weeding', hint: 'Autonomous weeding robot removing weeds between crop rows' },
  { id: 'manual', label: 'Kinematics', hint: 'Drag the effector around and explore the workspace' },
];
const SPEEDS = [0.25, 0.5, 1, 2, 4];

export function TopBar() {
  const { mode, setMode, playing, timeScale, set, restart, reroll, panelOpen } = useApp();
  const seed = useApp((s) => (s.mode === 'weeding' ? s.weeding.seed : s.pickPlace.seed));
  const simulated = mode !== 'manual';
  return (
    <header className="pointer-events-auto glass flex flex-wrap items-center gap-2 rounded-2xl px-3 py-2">
      <div className="mr-2 flex items-center gap-2.5 pl-1">
        <Logo />
        <div className="leading-tight">
          <div className="text-sm font-semibold text-white">Delta Robot Simulator</div>
          <div className="text-[11px] text-slate-400">kinematics · motion planning · scheduling</div>
        </div>
      </div>
      <nav className="flex rounded-xl bg-white/5 p-1" aria-label="Scenario">
        {MODES.map((m) => (
          <Tip key={m.id} content={m.hint}>
            <button
              type="button"
              onClick={() => mode !== m.id && setMode(m.id)}
              className={cn(
                'rounded-lg px-3 py-1.5 text-[13px] font-medium text-slate-400 transition-colors hover:text-white',
                mode === m.id && 'bg-white/10 text-white shadow-sm',
              )}
            >
              {m.label}
            </button>
          </Tip>
        ))}
      </nav>
      <div className="ml-auto flex items-center gap-1">
        {simulated && (
          <>
            <IconButton
              label={playing ? 'Pause (space)' : 'Play (space)'}
              onClick={() => set('playing', !playing)}
            >
              {playing ? <Pause size={16} /> : <Play size={16} />}
            </IconButton>
            <IconButton
              label="Single step"
              onClick={() => {
                set('playing', false);
                sim.loop?.stepOnce();
              }}
            >
              <StepForward size={16} />
            </IconButton>
            <div className="mx-1 flex rounded-lg bg-white/5 p-0.5">
              {SPEEDS.map((s) => (
                <button
                  key={s}
                  type="button"
                  onClick={() => set('timeScale', s)}
                  className={cn(
                    'tabular rounded-md px-1.5 py-1 font-mono text-[11px] text-slate-400 hover:text-white',
                    timeScale === s && 'bg-white/10 text-white',
                  )}
                >
                  {s}×
                </button>
              ))}
            </div>
            <IconButton label="Restart" onClick={restart}>
              <RotateCcw size={16} />
            </IconButton>
            <Tip content="New random scenario (seed)">
              <button
                type="button"
                onClick={reroll}
                className="inline-flex h-8 items-center gap-1.5 rounded-lg px-2 font-mono text-[11px] text-slate-400 hover:bg-white/10 hover:text-white"
              >
                <Dices size={15} />
                <span className="tabular">#{seed}</span>
              </button>
            </Tip>
            <IconButton label="Strategy benchmark" onClick={() => set('benchmarkOpen', true)}>
              <BarChart3 size={16} />
            </IconButton>
          </>
        )}
        <IconButton label="Settings" active={panelOpen} onClick={() => set('panelOpen', !panelOpen)}>
          <SlidersHorizontal size={16} />
        </IconButton>
      </div>
    </header>
  );
}

function Logo() {
  return (
    <svg viewBox="0 0 32 32" className="size-8" aria-hidden>
      <rect width="32" height="32" rx="8" fill="#10171e" stroke="rgba(255,255,255,0.08)" />
      <g stroke="#34d399" strokeWidth="2.2" strokeLinecap="round" fill="none">
        <path d="M7 8h18" />
        <path d="M9 8l-2 7 7 9" />
        <path d="M23 8l2 7-7 9" />
        <path d="M16 8v16" />
      </g>
      <circle cx="16" cy="24" r="2.6" fill="#34d399" />
    </svg>
  );
}
