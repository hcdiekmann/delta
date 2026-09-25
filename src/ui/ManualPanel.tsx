import { RotateCcw } from 'lucide-react';
import { useManual } from '@/state/manual';
import { PRESETS, type PresetName } from '@/core/kinematics/params';
import { fmt, rad2deg, cn } from '@/lib/utils';
import { IconButton, Section, Segmented, Slider } from './components/controls';

const REASONS = {
  unreachable: 'Out of reach: the rods cannot meet the target',
  jointLimit: 'Joint limit reached',
  ballJoint: 'Ball joint angle limit reached',
};

export function ManualPanel() {
  const m = useManual();
  const params = PRESETS[m.preset];
  return (
    <>
      <Section
        title="Robot"
        right={
          <IconButton label="Reset pose" onClick={m.reset}>
            <RotateCcw size={14} />
          </IconButton>
        }
      >
        <Segmented
          value={m.preset}
          onChange={(v: PresetName) => m.setPreset(v)}
          options={[
            { value: 'picker', label: 'Picker', hint: 'Upper arm 0.6 m, lower arm 1.0 m' },
            { value: 'weeder', label: 'Weeder', hint: 'Upper arm 0.4 m, lower arm 0.65 m' },
          ]}
        />
        <Segmented
          label="Control"
          value={m.mode}
          onChange={m.setMode}
          options={[
            {
              value: 'ik',
              label: 'Drag effector (IK)',
              hint: 'Inverse kinematics: move the gizmo, the joint angles follow',
            },
            {
              value: 'fk',
              label: 'Joint sliders (FK)',
              hint: 'Forward kinematics: set the motor angles directly',
            },
          ]}
        />
      </Section>
      <Section title={m.mode === 'fk' ? 'Joint angles' : 'State'}>
        {m.mode === 'fk' ? (
          [0, 1, 2].map((i) => (
            <Slider
              key={i}
              label={`Motor ${i + 1}`}
              value={rad2deg(m.theta[i as 0 | 1 | 2])}
              min={rad2deg(params.thetaMin)}
              max={rad2deg(params.thetaMax)}
              step={0.5}
              unit="°"
              digits={1}
              onChange={(v) => m.setJoint(i, (v * Math.PI) / 180)}
            />
          ))
        ) : (
          <div className="grid grid-cols-3 gap-1.5">
            {m.theta.map((t, i) => (
              <Readout key={i} label={`θ${i + 1}`} value={`${fmt(rad2deg(t), 1)}°`} />
            ))}
          </div>
        )}
        <div className="grid grid-cols-3 gap-1.5">
          <Readout label="x" value={fmt(m.flange.x * 1000, 0)} unit="mm" />
          <Readout label="y" value={fmt(m.flange.y * 1000, 0)} unit="mm" />
          <Readout label="z" value={fmt(m.flange.z * 1000, 0)} unit="mm" />
        </div>
        <div>
          <div className="mb-1 flex justify-between text-[12px] text-slate-400">
            <span>Conditioning</span>
            <span className="tabular font-mono">{fmt(m.conditioning, 2)}</span>
          </div>
          <div className="h-1.5 overflow-hidden rounded-full bg-white/10">
            <div
              className={cn(
                'h-full rounded-full transition-all',
                m.conditioning < 0.25 ? 'bg-warn' : 'bg-accent',
              )}
              style={{ width: `${Math.min(100, m.conditioning * 100)}%` }}
            />
          </div>
          <p className="mt-1 text-[11px] text-slate-500">
            Near 0 = close to a singularity (the robot loses stiffness).
          </p>
        </div>
        <div
          className={cn(
            'rounded-lg px-3 py-2 text-[12px]',
            m.status.ok ? 'bg-accent/10 text-accent' : 'bg-bad/10 text-bad',
          )}
        >
          {m.status.ok ? 'Reachable' : `${REASONS[m.status.reason]} (leg ${m.status.leg + 1})`}
        </div>
      </Section>
    </>
  );
}

function Readout({ label, value, unit }: { label: string; value: string; unit?: string }) {
  return (
    <div className="rounded-lg bg-white/[0.04] px-2 py-1.5">
      <div className="text-[10px] text-slate-500">{label}</div>
      <div className="tabular font-mono text-[13px] text-white">
        {value}
        {unit && <span className="ml-0.5 text-[10px] text-slate-500">{unit}</span>}
      </div>
    </div>
  );
}
