import type { PickPlaceConfig } from '@/core/scenarios/pickPlace/pickPlace';
import type { WeedingConfig } from '@/core/scenarios/weeding/weeding';
import type { Mode } from './store';

/**
 * Scenario settings live in the URL hash so a particular setup (including the seed) can be shared.
 * Format: #mode=weeding&speed=0.2&seed=42 ...
 */
interface UrlState {
  mode?: Mode;
  pickPlace?: Partial<PickPlaceConfig>;
  weeding?: Partial<WeedingConfig>;
}

const parseValue = (v: string): string | number | boolean =>
  v === 'true' ? true : v === 'false' ? false : v !== '' && !Number.isNaN(Number(v)) ? Number(v) : v;

export function readUrlState(): UrlState {
  if (typeof window === 'undefined') return {};
  const params = new URLSearchParams(window.location.hash.slice(1));
  const mode = params.get('mode') as Mode | null;
  const values: Record<string, string | number | boolean> = {};
  params.forEach((v, k) => {
    if (k !== 'mode') values[k] = parseValue(v);
  });
  if (mode === 'weeding') return { mode, weeding: values as Partial<WeedingConfig> };
  if (mode === 'pickPlace') return { mode, pickPlace: values as Partial<PickPlaceConfig> };
  if (mode === 'manual') return { mode };
  return {};
}

export function writeUrlState(mode: Mode, config: object | null) {
  const params = new URLSearchParams({ mode });
  if (config) for (const [k, v] of Object.entries(config)) params.set(k, String(v));
  window.history.replaceState(null, '', `#${params.toString()}`);
}
