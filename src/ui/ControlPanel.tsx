import { Camera, Eye, Orbit, ScanLine } from 'lucide-react';
import { useApp, type CameraMode } from '@/state/store';
import { STRATEGIES, type Strategy } from '@/core/scheduling/scheduler';
import { FIELD_PRESETS, type FieldPresetId } from '@/core/scenarios/weeding/field';
import { Section, Segmented, Slider, Switch } from './components/controls';
import { ManualPanel } from './ManualPanel';

export function ControlPanel() {
  const mode = useApp((s) => s.mode);
  return (
    <aside className="pointer-events-auto glass flex max-h-full w-[300px] flex-col overflow-hidden rounded-2xl">
      <div className="overflow-y-auto overscroll-contain">
        {mode === 'pickPlace' && <PickPlaceSettings />}
        {mode === 'weeding' && <WeedingSettings />}
        {mode === 'manual' && <ManualPanel />}
        <ViewSettings />
      </div>
    </aside>
  );
}

function StrategyPicker({ value, onChange }: { value: Strategy; onChange: (s: Strategy) => void }) {
  return (
    <Segmented
      label="Scheduling strategy"
      hint="Which detected item the robot goes for next"
      value={value}
      onChange={onChange}
      options={STRATEGIES.map((s) => ({
        value: s.id,
        label: s.id === 'edf' ? 'EDF' : s.id === 'fifo' ? 'FIFO' : 'Nearest',
        hint: `${s.label}: ${s.hint}`,
      }))}
    />
  );
}

function PickPlaceSettings() {
  const c = useApp((s) => s.pickPlace);
  const set = useApp((s) => s.setPickPlace);
  return (
    <>
      <Section title="Line">
        <Slider
          label="Conveyor speed"
          value={c.conveyorSpeed}
          min={0.1}
          max={0.6}
          step={0.01}
          unit=" m/s"
          onChange={(v) => set({ conveyorSpeed: v })}
        />
        <Slider
          label="Product rate"
          hint="Average number of products arriving per second (Poisson process)"
          value={c.spawnRate}
          min={0.3}
          max={2.5}
          step={0.05}
          unit=" /s"
          onChange={(v) => set({ spawnRate: v })}
        />
        <Segmented
          label="Product mix"
          value={c.mix}
          onChange={(v) => set({ mix: v })}
          options={[
            { value: 'mixed', label: 'Mixed' },
            { value: 'boxes', label: 'Boxes only' },
          ]}
        />
        <Switch
          label="Align products in trays"
          hint="Rotate the 4th axis so every product ends up in the same orientation"
          checked={c.alignProducts}
          onChange={(v) => set({ alignProducts: v })}
        />
      </Section>
      <Section title="Placing">
        <Segmented
          value={c.placeMode}
          onChange={(v) => set({ placeMode: v }, true)}
          options={[
            {
              value: 'trayConveyor',
              label: 'Tray conveyor',
              hint: 'Trays move past on a second belt: placing is another moving-target intercept',
            },
            {
              value: 'staticTrays',
              label: 'Static trays',
              hint: 'Fixed trays next to the belt, swapped when full',
            },
          ]}
        />
        {c.placeMode === 'trayConveyor' && (
          <>
            <Segmented
              label="Tray direction"
              value={c.trayFlow}
              onChange={(v) => set({ trayFlow: v }, true)}
              options={[
                { value: 'counter', label: 'Counter-flow' },
                { value: 'co', label: 'Co-flow' },
              ]}
            />
            <Switch
              label="Sync tray belt to product rate"
              hint="Run the tray belt just fast enough to supply one slot per product"
              checked={c.traySync}
              onChange={(v) => set({ traySync: v })}
            />
            {!c.traySync && (
              <Slider
                label="Tray belt speed"
                value={c.traySpeed}
                min={0.02}
                max={0.4}
                step={0.01}
                unit=" m/s"
                onChange={(v) => set({ traySpeed: v })}
              />
            )}
          </>
        )}
      </Section>
      <Section title="Robot">
        <StrategyPicker value={c.strategy} onChange={(v) => set({ strategy: v })} />
      </Section>
    </>
  );
}

function WeedingSettings() {
  const c = useApp((s) => s.weeding);
  const set = useApp((s) => s.setWeeding);
  return (
    <>
      <Section title="Field">
        <Segmented
          value={c.field}
          onChange={(v: FieldPresetId) => set({ field: v }, true)}
          options={Object.values(FIELD_PRESETS).map((p) => ({
            value: p.id,
            label: p.label,
            hint: p.description,
          }))}
        />
        <Slider
          label="Weed density"
          value={c.weedDensity}
          min={1}
          max={15}
          step={0.5}
          unit=" /m²"
          digits={1}
          onChange={(v) => set({ weedDensity: v }, true)}
        />
        <Slider
          label="Vehicle speed"
          value={c.speed}
          min={0.03}
          max={0.4}
          step={0.01}
          unit=" m/s"
          onChange={(v) => set({ speed: v })}
        />
        <Segmented
          label="Run length"
          value={String(c.runLength)}
          onChange={(v) => set({ runLength: Number(v) }, true)}
          options={[
            { value: '0', label: 'Endless' },
            { value: '10', label: '10 m' },
            { value: '25', label: '25 m' },
          ]}
        />
      </Section>
      <Section title="Vision">
        <Slider
          label="Classifier threshold"
          hint="Detections with a weed confidence above this are treated as weeds. Lower: fewer weeds missed, more crops pulled."
          value={c.threshold}
          min={0.05}
          max={0.95}
          step={0.01}
          onChange={(v) => set({ threshold: v })}
        />
      </Section>
      <Section title="Crop safety">
        <Slider
          label="Extra clearance"
          hint="Distance kept between the gripper and crop canopies"
          value={c.clearance * 100}
          min={0}
          max={5}
          step={0.1}
          unit=" cm"
          digits={1}
          onChange={(v) => set({ clearance: v / 100 })}
        />
        <Segmented
          label="Weeds close to crops"
          value={c.clearancePolicy}
          onChange={(v) => set({ clearancePolicy: v })}
          options={[
            { value: 'skip', label: 'Skip', hint: 'Leave weeds that grow too close to a crop' },
            { value: 'attempt', label: 'Attempt', hint: 'Grip anyway, only avoiding the crop stem' },
          ]}
        />
      </Section>
      <Section title="Robot">
        <StrategyPicker value={c.strategy} onChange={(v) => set({ strategy: v })} />
        <Segmented
          label="Weed disposal"
          value={c.disposal}
          onChange={(v) => set({ disposal: v }, true)}
          options={[
            { value: 'bin', label: 'Bin', hint: 'Collect weeds in a bin on the vehicle' },
            { value: 'furrow', label: 'Furrow', hint: 'Drop pulled weeds into the furrow to dry out' },
          ]}
        />
      </Section>
    </>
  );
}

const CAMERAS: { value: CameraMode; label: React.ReactNode; hint: string }[] = [
  { value: 'follow', label: <Orbit size={14} className="mx-auto" />, hint: 'Overview (follows the vehicle)' },
  { value: 'robot', label: <Camera size={14} className="mx-auto" />, hint: 'Close-up on the robot' },
  { value: 'top', label: <ScanLine size={14} className="mx-auto" />, hint: 'Top-down view' },
  { value: 'free', label: <Eye size={14} className="mx-auto" />, hint: 'Free camera (does not follow)' },
];

function ViewSettings() {
  const { mode, camera, view, quality, set, setView } = useApp();
  return (
    <Section title="View">
      <Segmented label="Camera" value={camera} onChange={(v) => set('camera', v)} options={CAMERAS} />
      {mode !== 'manual' && (
        <>
          <Switch
            label="Workspace envelope"
            hint="Reachable volume of the tool tip"
            checked={view.workspace}
            onChange={(v) => setView({ workspace: v })}
          />
          <Switch label="Planned path" checked={view.path} onChange={(v) => setView({ path: v })} />
        </>
      )}
      {mode === 'weeding' && (
        <>
          <Switch
            label="Camera field of view"
            checked={view.vision}
            onChange={(v) => setView({ vision: v })}
          />
          <Switch
            label="Detection markers"
            checked={view.markers}
            onChange={(v) => setView({ markers: v })}
          />
        </>
      )}
      <Segmented
        label="Graphics"
        value={quality}
        onChange={(v) => set('quality', v)}
        options={[
          { value: 'high', label: 'High', hint: 'Shadows, ambient occlusion and bloom' },
          { value: 'low', label: 'Fast', hint: 'For slower devices' },
        ]}
      />
    </Section>
  );
}
