import { useMemo } from 'react';
import { useFrame } from '@react-three/fiber';
import * as THREE from 'three';
import type { Vec3 } from '@/core/math/vec3';

const MAX_POINTS = 64;

/** Planned tool path of the current move (world coordinates, inside ZUpRoot). */
export function PathPreview({
  read,
  color = '#fbbf24',
}: {
  read: () => { base: Vec3; points: Vec3[] } | null;
  color?: string;
}) {
  const line = useMemo(() => {
    const g = new THREE.BufferGeometry();
    g.setAttribute('position', new THREE.BufferAttribute(new Float32Array(MAX_POINTS * 3), 3));
    const m = new THREE.LineDashedMaterial({
      color,
      dashSize: 0.02,
      gapSize: 0.015,
      transparent: true,
      opacity: 0.9,
    });
    const l = new THREE.Line(g, m);
    l.frustumCulled = false;
    return l;
  }, [color]);

  useFrame(() => {
    const d = read();
    const attr = line.geometry.getAttribute('position') as THREE.BufferAttribute;
    const pts = d?.points ?? [];
    const n = Math.min(pts.length, MAX_POINTS);
    for (let k = 0; k < n; k++) {
      const p = pts[k]!;
      attr.setXYZ(k, p.x + d!.base.x, p.y + d!.base.y, p.z + d!.base.z);
    }
    attr.needsUpdate = true;
    line.geometry.setDrawRange(0, n);
    line.computeLineDistances();
  });

  return <primitive object={line} />;
}
