import { computeWorkspace, type Workspace } from '@/core/kinematics/workspace';
import type { DeltaParams } from '@/core/kinematics/params';

const cache = new WeakMap<DeltaParams, Workspace | null>();

/** Workspace of a robot (with the planning margin), computed once per parameter set. */
export function workspaceFor(params: DeltaParams): Workspace | null {
  if (!cache.has(params))
    cache.set(params, computeWorkspace(params, { slices: 36, azimuths: 72, margin: 0.05 }));
  return cache.get(params)!;
}
