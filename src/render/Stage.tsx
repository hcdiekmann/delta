import { useRef } from 'react';
import { useFrame } from '@react-three/fiber';
import { Environment, Lightformer } from '@react-three/drei';
import { Bloom, EffectComposer, N8AO, SMAA, Vignette } from '@react-three/postprocessing';
import * as THREE from 'three';
import type { Vec3 } from '@/core/math/vec3';
import { toThree } from './helpers';

const tmp = new THREE.Vector3();

/** Lights, environment and post-processing. The key light follows `anchor` so shadows stay sharp. */
export function Stage({ anchor, quality }: { anchor: () => Vec3; quality: 'high' | 'low' }) {
  const key = useRef<THREE.DirectionalLight>(null);
  useFrame(() => {
    const l = key.current;
    if (!l) return;
    toThree(anchor(), tmp);
    l.position.set(tmp.x + 3.5, tmp.y + 7, tmp.z + 2.5);
    l.target.position.copy(tmp);
    l.target.updateMatrixWorld();
  });
  const high = quality === 'high';
  return (
    <>
      <color attach="background" args={['#0b1015']} />
      <fog attach="fog" args={['#0b1015', 9, 26]} />
      <hemisphereLight args={['#e0ecff', '#3a3026', 1.4]} />
      <directionalLight
        ref={key}
        intensity={3.2}
        color="#fff4e6"
        castShadow={high}
        shadow-mapSize={[2048, 2048]}
        shadow-bias={-0.0004}
        shadow-normalBias={0.02}
        shadow-camera-left={-4}
        shadow-camera-right={4}
        shadow-camera-top={4}
        shadow-camera-bottom={-4}
        shadow-camera-near={0.5}
        shadow-camera-far={20}
      />
      <directionalLight position={[-6, 4, -5]} intensity={0.9} color="#93c5fd" />
      <Environment resolution={64} frames={1} environmentIntensity={1.3}>
        <Lightformer intensity={1.2} position={[0, 6, -6]} scale={[12, 4, 1]} color="#dbeafe" />
        <Lightformer
          intensity={0.8}
          position={[-6, 3, 3]}
          rotation-y={Math.PI / 2}
          scale={[8, 3, 1]}
          color="#fde68a"
        />
        <Lightformer
          intensity={0.5}
          position={[6, 2, 3]}
          rotation-y={-Math.PI / 2}
          scale={[8, 3, 1]}
          color="#a7f3d0"
        />
      </Environment>
      {high && (
        <EffectComposer multisampling={0}>
          <N8AO aoRadius={0.35} intensity={1.6} distanceFalloff={0.6} halfRes />
          <Bloom luminanceThreshold={0.95} intensity={0.35} mipmapBlur />
          <SMAA />
          <Vignette offset={0.25} darkness={0.55} />
        </EffectComposer>
      )}
    </>
  );
}
