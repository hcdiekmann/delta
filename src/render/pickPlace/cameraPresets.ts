import { add } from '@/core/math/vec3';
import { LAYOUT, type PickPlaceScenario } from '@/core/scenarios/pickPlace/pickPlace';
import type { CameraPresets } from '../CameraRig';

/** Camera presets for the scene (world, Z-up): [position, target]. */
export function pickPlaceCameraPresets(scenario: PickPlaceScenario): CameraPresets {
  const b = scenario.robotBase();
  const work = add(b, { x: 0.1, y: 0, z: -0.95 });
  return {
    follow: { position: { x: 1.3, y: -3.7, z: 2.5 }, target: work },
    robot: { position: { x: 0.9, y: -1.2, z: 1.35 }, target: { x: 0, y: 0, z: 1.0 } },
    top: { position: { x: 0.01, y: -0.2, z: 4.2 }, target: { x: 0, y: 0, z: LAYOUT.beltHeight } },
    free: { position: { x: 1.3, y: -3.7, z: 2.5 }, target: work },
  };
}
