import { PickPlaceScenario } from '@/core/scenarios/pickPlace/pickPlace';
import { WeedingScenario } from '@/core/scenarios/weeding/weeding';
import { SimLoop } from '@/core/sim/simLoop';
import { useApp } from './store';

export type ActiveScenario = PickPlaceScenario | WeedingScenario;

/**
 * Owner of the running simulation. Per-frame data is read directly from here inside useFrame
 * (never through React state) so the UI only re-renders when settings change.
 */
class SimHandle {
  scenario: ActiveScenario | null = null;
  loop: SimLoop | null = null;
  private listeners = new Set<() => void>();

  build() {
    const s = useApp.getState();
    if (s.mode === 'pickPlace') this.scenario = new PickPlaceScenario(s.pickPlace);
    else if (s.mode === 'weeding') this.scenario = new WeedingScenario(s.weeding);
    else this.scenario = null;
    const scenario = this.scenario;
    this.loop = scenario ? new SimLoop((dt) => scenario.step(dt)) : null;
    this.listeners.forEach((l) => l());
  }

  /** Subscribe to scenario rebuilds */
  subscribe(fn: () => void) {
    this.listeners.add(fn);
    return () => {
      this.listeners.delete(fn);
    };
  }
}

export const sim = new SimHandle();

/** Keep the simulation in sync with the settings store. */
export function connectSimToStore() {
  let last = useApp.getState();
  sim.build();
  return useApp.subscribe((s) => {
    if (s.epoch !== last.epoch || s.mode !== last.mode) sim.build();
    else if (sim.scenario instanceof PickPlaceScenario && s.pickPlace !== last.pickPlace)
      sim.scenario.configure(s.pickPlace);
    else if (sim.scenario instanceof WeedingScenario && s.weeding !== last.weeding)
      sim.scenario.configure(s.weeding);
    last = s;
  });
}
