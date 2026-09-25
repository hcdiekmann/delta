import { create } from 'zustand';
import { vec3, lerp, type Vec3 } from '@/core/math/vec3';
import {
  conditioning,
  forwardKinematics,
  homePosition,
  inverseKinematics,
  type IkFailure,
  type Joints,
} from '@/core/kinematics/kinematics';
import { PRESETS, type PresetName } from '@/core/kinematics/params';

export type ManualMode = 'ik' | 'fk';

export interface ManualState {
  preset: PresetName;
  mode: ManualMode;
  /** Requested flange position (robot frame) */
  target: Vec3;
  /** Actual flange position (clamped to the workspace) */
  flange: Vec3;
  theta: Joints;
  status: { ok: true } | { ok: false; reason: IkFailure; leg: number };
  conditioning: number;
  /** Bumped to move the gizmo back onto the robot */
  gizmoKey: number;
  setPreset(p: PresetName): void;
  setMode(m: ManualMode): void;
  setTarget(p: Vec3): void;
  setJoint(i: number, value: number): void;
  reset(): void;
}

function initial(preset: PresetName) {
  const params = PRESETS[preset];
  const home = homePosition(params);
  const ik = inverseKinematics(params, home);
  const theta: Joints = ik.ok ? ik.theta : [0, 0, 0];
  return {
    preset,
    target: home,
    flange: home,
    theta,
    status: { ok: true as const },
    conditioning: conditioning(params, theta, home),
  };
}

/**
 * Interactive kinematics playground. Out-of-reach targets are clamped to the boundary by bisecting
 * between the last valid position and the requested one, and the reason is reported.
 */
export const useManual = create<ManualState>()((set, get) => ({
  ...initial('picker'),
  mode: 'ik',
  gizmoKey: 0,
  setPreset: (preset) => set((s) => ({ ...initial(preset), gizmoKey: s.gizmoKey + 1 })),
  setMode: (mode) => set((s) => ({ mode, target: s.flange, gizmoKey: s.gizmoKey + 1 })),
  setTarget: (target) => {
    const s = get();
    const params = PRESETS[s.preset];
    const ik = inverseKinematics(params, target);
    if (ik.ok) {
      set({
        target,
        flange: target,
        theta: ik.theta,
        status: { ok: true },
        conditioning: conditioning(params, ik.theta, target),
      });
      return;
    }
    let good = s.flange;
    let bad = target;
    for (let k = 0; k < 24; k++) {
      const mid = lerp(good, bad, 0.5);
      if (inverseKinematics(params, mid).ok) good = mid;
      else bad = mid;
    }
    const clamped = inverseKinematics(params, good);
    set({
      target,
      flange: good,
      theta: clamped.ok ? clamped.theta : s.theta,
      status: { ok: false, reason: ik.reason, leg: ik.leg },
      conditioning: clamped.ok ? conditioning(params, clamped.theta, good) : 0,
    });
  },
  setJoint: (i, value) => {
    const s = get();
    const params = PRESETS[s.preset];
    const theta = s.theta.map((t, k) => (k === i ? value : t)) as unknown as Joints;
    const p = forwardKinematics(params, theta);
    if (!p) return;
    const check = inverseKinematics(params, p);
    set({
      theta,
      flange: p,
      target: p,
      status: check.ok ? { ok: true } : { ok: false, reason: check.reason, leg: check.leg },
      conditioning: conditioning(params, theta, p),
    });
  },
  reset: () => set((s) => ({ ...initial(s.preset), gizmoKey: s.gizmoKey + 1 })),
}));

export const MANUAL_BASE = vec3(0, 0, 1.9);
