import type { ReactNode } from 'react';

/**
 * The simulation core works in a Z-up frame; three.js is Y-up. Everything inside this group can
 * use domain coordinates directly: (x, y, z) maps to three.js (x, z, -y).
 */
export function ZUpRoot({ children }: { children: ReactNode }) {
  return <group rotation-x={-Math.PI / 2}>{children}</group>;
}
