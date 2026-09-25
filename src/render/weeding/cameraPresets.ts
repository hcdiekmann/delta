import type { WeedingScenario } from '@/core/scenarios/weeding/weeding';
import type { CameraPresets } from '../CameraRig';

/** Camera presets relative to the vehicle (world, Z-up). */
export function weedingCameraPresets(scenario: WeedingScenario): CameraPresets {
  const h = scenario.preset.bedHeight;
  const big = scenario.preset.robot === 'picker';
  const k = big ? 1.4 : 1;
  return {
    follow: { position: { x: 2.7 * k, y: -2.1 * k, z: h + 2.1 * k }, target: { x: 0.25, y: 0, z: h + 0.3 } },
    robot: { position: { x: 1.35 * k, y: -0.3 * k, z: h + 0.6 * k }, target: { x: 0.05, y: 0, z: h + 0.25 } },
    top: { position: { x: 0.3, y: -0.01, z: h + 3.4 * k }, target: { x: 0.3, y: 0, z: h } },
    free: { position: { x: 2.7 * k, y: -2.1 * k, z: h + 2.1 * k }, target: { x: 0.25, y: 0, z: h + 0.3 } },
  };
}
