import { useFrame } from '@react-three/fiber';
import { sim } from '@/state/simHandle';
import { useApp } from '@/state/store';

/** Advances the simulation from the render loop with a fixed internal time step. */
export function SimDriver() {
  useFrame((_, delta) => {
    const loop = sim.loop;
    const scenario = sim.scenario;
    if (!loop || !scenario) return;
    const { playing, timeScale } = useApp.getState();
    loop.paused = !playing || scenario.finished;
    loop.timeScale = timeScale;
    loop.advance(delta);
  });
  return null;
}
