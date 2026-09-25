import { create } from 'zustand';
import { DEFAULT_PICK_PLACE, type PickPlaceConfig } from '@/core/scenarios/pickPlace/pickPlace';
import { DEFAULT_WEEDING, type WeedingConfig } from '@/core/scenarios/weeding/weeding';
import type { PresetName } from '@/core/kinematics/params';
import { readUrlState } from './urlState';

export type Mode = 'pickPlace' | 'weeding' | 'manual';
export type CameraMode = 'follow' | 'robot' | 'top' | 'free';
export type ChartKind = 'theta' | 'omega' | 'tau';

export interface ViewOptions {
  workspace: boolean;
  path: boolean;
  vision: boolean;
  markers: boolean;
}

export interface AppState {
  mode: Mode;
  pickPlace: PickPlaceConfig;
  weeding: WeedingConfig;
  manualPreset: PresetName;
  playing: boolean;
  timeScale: number;
  camera: CameraMode;
  view: ViewOptions;
  chart: ChartKind;
  panelOpen: boolean;
  quality: 'high' | 'low';
  benchmarkOpen: boolean;
  /** Incremented whenever the simulation has to be rebuilt */
  epoch: number;

  setMode(mode: Mode): void;
  /** Update settings; `rebuild` restarts the simulation (for structural changes) */
  setPickPlace(patch: Partial<PickPlaceConfig>, rebuild?: boolean): void;
  setWeeding(patch: Partial<WeedingConfig>, rebuild?: boolean): void;
  set<K extends keyof AppState>(key: K, value: AppState[K]): void;
  setView(patch: Partial<ViewOptions>): void;
  restart(): void;
  reroll(): void;
}

const initial = readUrlState();

export const useApp = create<AppState>()((set, get) => ({
  mode: initial.mode ?? 'pickPlace',
  pickPlace: { ...DEFAULT_PICK_PLACE, ...initial.pickPlace },
  weeding: { ...DEFAULT_WEEDING, ...initial.weeding },
  manualPreset: 'picker',
  playing: true,
  timeScale: 1,
  camera: 'follow',
  view: { workspace: false, path: true, vision: true, markers: true },
  chart: 'theta',
  panelOpen: typeof window === 'undefined' || window.innerWidth > 900,
  quality: 'high',
  benchmarkOpen: false,
  epoch: 0,

  setMode: (mode) => set({ mode, epoch: get().epoch + 1, camera: 'follow', playing: true }),
  setPickPlace: (patch, rebuild = false) =>
    set((s) => ({ pickPlace: { ...s.pickPlace, ...patch }, epoch: rebuild ? s.epoch + 1 : s.epoch })),
  setWeeding: (patch, rebuild = false) =>
    set((s) => ({ weeding: { ...s.weeding, ...patch }, epoch: rebuild ? s.epoch + 1 : s.epoch })),
  set: (key, value) => set({ [key]: value } as Partial<AppState>),
  setView: (patch) => set((s) => ({ view: { ...s.view, ...patch } })),
  restart: () => set((s) => ({ epoch: s.epoch + 1, playing: true })),
  reroll: () => {
    const seed = Math.floor(Math.random() * 1e6);
    set((s) => ({
      pickPlace: { ...s.pickPlace, seed },
      weeding: { ...s.weeding, seed },
      epoch: s.epoch + 1,
      playing: true,
    }));
  },
}));
