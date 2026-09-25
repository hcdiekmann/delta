import { useEffect, useMemo, useState } from 'react';
import { Canvas } from '@react-three/fiber';
import { Grid, PerformanceMonitor } from '@react-three/drei';
import * as THREE from 'three';
import { PickPlaceScenario } from '@/core/scenarios/pickPlace/pickPlace';
import { WeedingScenario } from '@/core/scenarios/weeding/weeding';
import { vec3, type Vec3 } from '@/core/math/vec3';
import { sim } from '@/state/simHandle';
import { useApp } from '@/state/store';
import { MANUAL_BASE } from '@/state/manual';
import { ZUpRoot } from './ZUpRoot';
import { Stage } from './Stage';
import { CameraRig, type CameraPresets } from './CameraRig';
import { SimDriver } from './SimDriver';
import { PickPlaceScene } from './pickPlace/PickPlaceScene';
import { pickPlaceCameraPresets } from './pickPlace/cameraPresets';
import { WeedingScene } from './weeding/WeedingScene';
import { weedingCameraPresets } from './weeding/cameraPresets';
import { ManualScene } from './manual/ManualScene';

const ORIGIN = vec3();
const MANUAL_PRESETS: CameraPresets = {
  follow: { position: vec3(2.6, -3.2, 2.7), target: vec3(0, 0, MANUAL_BASE.z - 0.9) },
  robot: { position: vec3(0.8, -1.1, 1.5), target: vec3(0, 0, MANUAL_BASE.z - 0.7) },
  top: { position: vec3(0.01, -0.1, 4.5), target: vec3(0, 0, 0.8) },
  free: { position: vec3(2.6, -3.2, 2.7), target: vec3(0, 0, MANUAL_BASE.z - 0.9) },
};

function useScenario() {
  const [, force] = useState(0);
  useEffect(() => sim.subscribe(() => force((n) => n + 1)), []);
  return sim.scenario;
}

function SceneContent() {
  const mode = useApp((s) => s.mode);
  const quality = useApp((s) => s.quality);
  const scenario = useScenario();

  const { presets, anchor } = useMemo((): { presets: CameraPresets; anchor: () => Vec3 } => {
    if (scenario instanceof WeedingScenario)
      return { presets: weedingCameraPresets(scenario), anchor: () => vec3(scenario.vehicleX, 0, 0) };
    if (scenario instanceof PickPlaceScenario)
      return { presets: pickPlaceCameraPresets(scenario), anchor: () => ORIGIN };
    return { presets: MANUAL_PRESETS, anchor: () => ORIGIN };
  }, [scenario]);

  return (
    <>
      <Stage anchor={anchor} quality={quality} />
      <CameraRig presets={presets} anchor={anchor} />
      <SimDriver />
      <ZUpRoot>
        {mode === 'manual' && <ManualScene />}
        {mode === 'pickPlace' && scenario instanceof PickPlaceScenario && (
          <PickPlaceScene scenario={scenario} />
        )}
        {mode === 'weeding' && scenario instanceof WeedingScenario && <WeedingScene scenario={scenario} />}
        {mode !== 'weeding' && <Floor />}
      </ZUpRoot>
    </>
  );
}

function Floor() {
  return (
    <group>
      <mesh receiveShadow position-z={-0.001}>
        <planeGeometry args={[60, 60]} />
        <meshStandardMaterial color="#1a222b" roughness={0.95} />
      </mesh>
      <Grid
        rotation-x={Math.PI / 2}
        position-z={0.001}
        args={[40, 40]}
        cellSize={0.25}
        cellThickness={0.6}
        cellColor="#1f2833"
        sectionSize={1}
        sectionThickness={1}
        sectionColor="#2b3643"
        fadeDistance={18}
        infiniteGrid
      />
    </group>
  );
}

export function Scene() {
  const setQuality = useApp((s) => s.set);
  return (
    <Canvas
      shadows="percentage"
      dpr={[1, 2]}
      camera={{ fov: 40, near: 0.05, far: 80, position: [3, 2.5, 3] }}
      gl={{ antialias: false, powerPreference: 'high-performance', toneMapping: THREE.ACESFilmicToneMapping }}
    >
      <PerformanceMonitor onDecline={() => setQuality('quality', 'low')} />
      <SceneContent />
    </Canvas>
  );
}
