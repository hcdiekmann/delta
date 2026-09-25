import { useEffect, useRef, type ComponentRef } from 'react';
import { useFrame, useThree } from '@react-three/fiber';
import { OrbitControls } from '@react-three/drei';
import * as THREE from 'three';
import { add, type Vec3 } from '@/core/math/vec3';
import { useApp, type CameraMode } from '@/state/store';
import { toThree } from './helpers';

export type CameraPresets = Record<CameraMode, { position: Vec3; target: Vec3 }>;

const easeInOut = (t: number) => (t < 0.5 ? 4 * t * t * t : 1 - Math.pow(-2 * t + 2, 3) / 2);

/**
 * Orbit controls that can track a moving anchor (the weeding vehicle): each frame the camera and
 * its target are shifted by the anchor's motion, so the user can still orbit and zoom freely.
 * Switching camera mode animates to the preset view relative to the anchor.
 */
export function CameraRig({ presets, anchor }: { presets: CameraPresets; anchor: () => Vec3 }) {
  const controls = useRef<ComponentRef<typeof OrbitControls>>(null);
  const camera = useThree((s) => s.camera);
  const mode = useApp((s) => s.camera);
  const last = useRef<THREE.Vector3 | null>(null);
  const anim = useRef<{ t: number; fromPos: THREE.Vector3; fromTarget: THREE.Vector3; snap: boolean } | null>(
    null,
  );

  useEffect(() => {
    const c = controls.current;
    anim.current = {
      t: 0,
      fromPos: camera.position.clone(),
      fromTarget: c ? c.target.clone() : new THREE.Vector3(),
      snap: last.current === null,
    };
  }, [mode, presets, camera]);

  useFrame((_, delta) => {
    const c = controls.current;
    if (!c) return;
    const a = toThree(anchor());
    if (last.current && mode !== 'free') {
      const d = a.clone().sub(last.current);
      camera.position.add(d);
      c.target.add(d);
    }
    last.current = a;
    const an = anim.current;
    if (an) {
      const preset = presets[mode];
      const anchorZ = anchor();
      const toPos = toThree(add(anchorZ, preset.position));
      const toTarget = toThree(add(anchorZ, preset.target));
      an.t = an.snap ? 1 : Math.min(1, an.t + delta / 0.9);
      const e = easeInOut(an.t);
      camera.position.lerpVectors(an.fromPos, toPos, e);
      c.target.lerpVectors(an.fromTarget, toTarget, e);
      if (an.t >= 1) anim.current = null;
    }
    c.update();
  });

  return (
    <OrbitControls
      ref={controls}
      makeDefault
      enableDamping
      dampingFactor={0.12}
      minDistance={0.4}
      maxDistance={14}
      maxPolarAngle={Math.PI * 0.49}
    />
  );
}
