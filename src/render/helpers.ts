import * as THREE from 'three';
import type { Vec3 } from '@/core/math/vec3';

/** Domain (Z-up) vector to three.js world (Y-up) coordinates. Only needed outside <ZUpRoot>. */
export const toThree = (v: Vec3, out = new THREE.Vector3()) => out.set(v.x, v.z, -v.y);

export const v3 = (v: Vec3, out = new THREE.Vector3()) => out.set(v.x, v.y, v.z);

const UP = new THREE.Vector3(0, 1, 0);
const tmpDir = new THREE.Vector3();

/**
 * Place a unit primitive whose long axis is +Y (cylinder or box of height 1) between two points.
 */
export function placeBetween(obj: THREE.Object3D, a: THREE.Vector3, b: THREE.Vector3, w: number, d = w) {
  tmpDir.subVectors(b, a);
  const len = tmpDir.length();
  obj.position.addVectors(a, b).multiplyScalar(0.5);
  if (len > 1e-9) obj.quaternion.setFromUnitVectors(UP, tmpDir.divideScalar(len));
  obj.scale.set(w, len, d);
}

export const PALETTE = {
  frame: '#8b95a3',
  frameDark: '#3a4450',
  graphite: '#262d36',
  white: '#e8ecf1',
  accent: '#34d399',
  accentDark: '#0f9f6e',
  sky: '#38bdf8',
  warn: '#fbbf24',
  bad: '#fb7185',
  soil: '#6b4a32',
  soilDark: '#523826',
  crop: '#4ade80',
  weed: '#a3a635',
  belt: '#1d232b',
};
