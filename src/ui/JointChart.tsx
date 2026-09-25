import { useEffect, useRef } from 'react';
import uPlot from 'uplot';
import { useApp, type ChartKind } from '@/state/store';
import { sim } from '@/state/simHandle';
import type { Channel } from '@/core/robot/telemetry';
import { Segmented } from './components/controls';
import { SERIES_COLORS } from '@/lib/palette';

const SERIES: Record<ChartKind, { channels: Channel[]; label: string; unit: string; scale: number }> = {
  theta: { channels: ['theta0', 'theta1', 'theta2'], label: 'Joint angle', unit: '°', scale: 180 / Math.PI },
  omega: {
    channels: ['omega0', 'omega1', 'omega2'],
    label: 'Joint speed',
    unit: '°/s',
    scale: 180 / Math.PI,
  },
  tau: { channels: ['tau0', 'tau1', 'tau2'], label: 'Motor torque', unit: 'Nm', scale: 1 },
};
const COLORS = SERIES_COLORS;
const WINDOW = 6;

/** Streaming chart of the robot's joint signals (last few seconds). */
export function JointChart() {
  const kind = useApp((s) => s.chart);
  const set = useApp((s) => s.set);
  const host = useRef<HTMLDivElement>(null);

  useEffect(() => {
    const el = host.current;
    if (!el) return;
    const cfg = SERIES[kind];
    const plot = new uPlot(
      {
        width: el.clientWidth,
        height: 150,
        padding: [8, 8, 0, 0],
        legend: { show: true, live: false },
        cursor: { show: false },
        scales: { x: { time: false } },
        axes: [
          {
            stroke: '#64748b',
            grid: { stroke: 'rgba(255,255,255,0.05)' },
            ticks: { show: false },
            values: (_u, vals) => vals.map((v) => `${v.toFixed(0)}s`),
            size: 24,
            font: '10px JetBrains Mono, monospace',
          },
          {
            stroke: '#64748b',
            grid: { stroke: 'rgba(255,255,255,0.05)' },
            ticks: { show: false },
            size: 44,
            font: '10px JetBrains Mono, monospace',
            values: (_u, vals) => vals.map((v) => `${v}`),
          },
        ],
        series: [
          {},
          ...cfg.channels.map((_, i) => ({
            label: `M${i + 1}`,
            stroke: COLORS[i],
            width: 1.5,
            points: { show: false },
          })),
        ],
      },
      [[], [], [], []],
      el,
    );
    const ro = new ResizeObserver(() => plot.setSize({ width: el.clientWidth, height: 150 }));
    ro.observe(el);
    let raf = 0;
    let last = 0;
    const tick = (now: number) => {
      raf = requestAnimationFrame(tick);
      if (now - last < 50) return;
      last = now;
      const s = sim.scenario;
      if (!s) return;
      const { t, series } = s.robot.telemetry.snapshot(cfg.channels);
      const t1 = t[t.length - 1] ?? 0;
      const from = t.findIndex((x) => x >= t1 - WINDOW);
      const start = Math.max(0, from);
      const xs = t.slice(start);
      plot.setData(
        [xs, ...series.map((ser) => ser.slice(start).map((v) => v * cfg.scale))] as uPlot.AlignedData,
        true,
      );
      plot.setScale('x', { min: Math.max(0, t1 - WINDOW), max: Math.max(WINDOW, t1) });
    };
    raf = requestAnimationFrame(tick);
    return () => {
      cancelAnimationFrame(raf);
      ro.disconnect();
      plot.destroy();
    };
  }, [kind]);

  const cfg = SERIES[kind];
  return (
    <div className="pointer-events-auto glass w-[420px] max-w-[calc(100vw-2rem)] rounded-2xl p-3">
      <div className="mb-1 flex items-center justify-between gap-3 px-1">
        <div>
          <span className="text-[11px] font-semibold tracking-[0.14em] text-slate-400 uppercase">
            {cfg.label}
          </span>
          <span className="ml-1.5 font-mono text-[11px] text-slate-500">[{cfg.unit}]</span>
        </div>
        <div className="w-48">
          <Segmented
            value={kind}
            onChange={(v) => set('chart', v)}
            options={[
              { value: 'theta', label: 'θ', hint: 'Joint angles' },
              { value: 'omega', label: 'ω', hint: 'Joint velocities' },
              { value: 'tau', label: 'τ', hint: 'Estimated motor torque (rigid body model, 38.5:1 gearbox)' },
            ]}
          />
        </div>
      </div>
      <div ref={host} className="w-full" />
    </div>
  );
}
