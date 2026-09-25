import { useMemo } from 'react';
import * as THREE from 'three';
import type { Workspace } from '@/core/kinematics/workspace';
import type { DeltaParams } from '@/core/kinematics/params';
import { workspaceFor } from './workspaceCache';

/** Loft the sampled workspace slices into a closed mesh (robot frame, flange positions). */
function buildGeometry(ws: Workspace, zOffset: number): THREE.BufferGeometry {
  const n = ws.slices[0]!.radii.length;
  const positions: number[] = [];
  const index: number[] = [];
  ws.slices.forEach((s) => {
    for (let k = 0; k < n; k++) {
      const a = (k / n) * Math.PI * 2;
      positions.push(s.radii[k]! * Math.cos(a), s.radii[k]! * Math.sin(a), s.z - zOffset);
    }
  });
  for (let j = 0; j < ws.slices.length - 1; j++) {
    for (let k = 0; k < n; k++) {
      const a = j * n + k;
      const b = j * n + ((k + 1) % n);
      const c = (j + 1) * n + k;
      const d = (j + 1) * n + ((k + 1) % n);
      index.push(a, c, b, b, c, d);
    }
  }
  // caps
  for (const [j, flip] of [
    [0, true],
    [ws.slices.length - 1, false],
  ] as const) {
    const centre = positions.length / 3;
    positions.push(0, 0, ws.slices[j]!.z - zOffset);
    for (let k = 0; k < n; k++) {
      const a = j * n + k;
      const b = j * n + ((k + 1) % n);
      if (flip) index.push(centre, b, a);
      else index.push(centre, a, b);
    }
  }
  const g = new THREE.BufferGeometry();
  g.setAttribute('position', new THREE.Float32BufferAttribute(positions, 3));
  g.setIndex(index);
  g.computeVertexNormals();
  return g;
}

/**
 * Translucent reachable volume of the tool tip. Drawn in the robot frame, i.e. place it inside a
 * group at the robot base.
 */
export function WorkspaceEnvelope({
  params,
  tool,
  color = '#38bdf8',
}: {
  params: DeltaParams;
  tool: number;
  color?: string;
}) {
  const built = useMemo(() => {
    const ws = workspaceFor(params);
    return ws ? { mesh: buildGeometry(ws, tool), rings: buildContours(ws, tool) } : null;
  }, [params, tool]);
  if (!built) return null;
  return (
    <group>
      <mesh geometry={built.mesh} renderOrder={2}>
        <meshStandardMaterial
          color={color}
          transparent
          opacity={0.09}
          depthWrite={false}
          side={THREE.DoubleSide}
          roughness={0.2}
        />
      </mesh>
      <lineSegments geometry={built.rings} renderOrder={3}>
        <lineBasicMaterial color={color} transparent opacity={0.45} depthWrite={false} />
      </lineSegments>
    </group>
  );
}

/** Contour rings every few slices plus a few vertical profile lines. */
function buildContours(ws: Workspace, zOffset: number): THREE.BufferGeometry {
  const pts: number[] = [];
  const n = ws.slices[0]!.radii.length;
  const at = (j: number, k: number) => {
    const s = ws.slices[j]!;
    const a = ((k % n) / n) * Math.PI * 2;
    return [s.radii[k % n]! * Math.cos(a), s.radii[k % n]! * Math.sin(a), s.z - zOffset];
  };
  for (let j = 0; j < ws.slices.length; j += 5)
    for (let k = 0; k < n; k++) pts.push(...at(j, k), ...at(j, k + 1));
  for (let k = 0; k < n; k += n / 12)
    for (let j = 0; j < ws.slices.length - 1; j++) pts.push(...at(j, k), ...at(j + 1, k));
  const g = new THREE.BufferGeometry();
  g.setAttribute('position', new THREE.Float32BufferAttribute(pts, 3));
  return g;
}

/** Outline of the reachable area at one height (e.g. on the conveyor or the ground). */
export function ReachFootprint({
  params,
  tool,
  z,
  color = '#38bdf8',
}: {
  params: DeltaParams;
  tool: number;
  /** Height of the tool tip in the robot frame */
  z: number;
  color?: string;
}) {
  const geometry = useMemo(() => {
    const ws = workspaceFor(params);
    if (!ws) return null;
    const flangeZ = z + tool;
    // interpolate between the two nearest slices
    const s = ws.slices;
    let j = 0;
    while (j < s.length - 2 && s[j + 1]!.z > flangeZ) j++;
    const a = s[j]!;
    const b = s[j + 1]!;
    const t = Math.min(1, Math.max(0, (a.z - flangeZ) / (a.z - b.z)));
    const n = a.radii.length;
    const shape = new THREE.Shape();
    const hole = new THREE.Path();
    for (let k = 0; k <= n; k++) {
      const ang = ((k % n) / n) * Math.PI * 2;
      const r = a.radii[k % n]! * (1 - t) + b.radii[k % n]! * t;
      const x = r * Math.cos(ang);
      const y = r * Math.sin(ang);
      if (k === 0) shape.moveTo(x, y);
      else shape.lineTo(x, y);
      const ri = Math.max(0, r - 0.012);
      if (k === 0) hole.moveTo(ri * Math.cos(ang), ri * Math.sin(ang));
      else hole.lineTo(ri * Math.cos(ang), ri * Math.sin(ang));
    }
    shape.holes.push(hole);
    return new THREE.ShapeGeometry(shape);
  }, [params, tool, z]);
  if (!geometry) return null;
  return (
    <mesh geometry={geometry} position-z={z + 0.003} renderOrder={4}>
      <meshBasicMaterial color={color} transparent opacity={0.55} depthWrite={false} />
    </mesh>
  );
}
