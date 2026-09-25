import { useEffect } from 'react';
import { Tooltip } from 'radix-ui';
import { Scene } from '@/render/Scene';
import { connectSimToStore } from '@/state/simHandle';
import { useApp } from '@/state/store';
import { writeUrlState } from '@/state/urlState';
import { TopBar } from './TopBar';
import { ControlPanel } from './ControlPanel';
import { StatsHud } from './StatsHud';
import { JointChart } from './JointChart';
import { RunSummary } from './RunSummary';
import { Benchmark } from './Benchmark';

export function App() {
  const mode = useApp((s) => s.mode);
  const panelOpen = useApp((s) => s.panelOpen);

  useEffect(() => connectSimToStore(), []);

  // shareable URL + keyboard shortcuts
  useEffect(
    () =>
      useApp.subscribe((s) =>
        writeUrlState(s.mode, s.mode === 'weeding' ? s.weeding : s.mode === 'pickPlace' ? s.pickPlace : null),
      ),
    [],
  );
  useEffect(() => {
    const onKey = (e: KeyboardEvent) => {
      if (e.target instanceof HTMLInputElement || e.target instanceof HTMLButtonElement) return;
      const s = useApp.getState();
      if (e.code === 'Space') {
        e.preventDefault();
        s.set('playing', !s.playing);
      } else if (e.key === 'r') s.restart();
      else if (e.key === '1') s.set('camera', 'follow');
      else if (e.key === '2') s.set('camera', 'robot');
      else if (e.key === '3') s.set('camera', 'top');
      else if (e.key === '4') s.set('camera', 'free');
    };
    window.addEventListener('keydown', onKey);
    return () => window.removeEventListener('keydown', onKey);
  }, []);

  return (
    <Tooltip.Provider>
      <div className="relative h-full w-full">
        <div className="absolute inset-0">
          <Scene />
        </div>
        <div className="pointer-events-none absolute inset-0 flex flex-col gap-3 p-3 sm:p-4">
          <TopBar />
          <div className="flex min-h-0 flex-1 items-start justify-end">{panelOpen && <ControlPanel />}</div>
          {mode !== 'manual' && (
            <div className="flex flex-wrap items-end justify-between gap-3">
              <StatsHud />
              <JointChart />
            </div>
          )}
          {mode === 'manual' && (
            <p className="glass pointer-events-auto self-start rounded-xl px-3 py-2 text-[12px] text-slate-400">
              Drag the arrows or planes of the gizmo to move the effector. The translucent volume is the
              reachable workspace.
            </p>
          )}
        </div>
        <RunSummary />
        <Benchmark />
      </div>
    </Tooltip.Provider>
  );
}
