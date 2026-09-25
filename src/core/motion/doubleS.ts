/**
 * Jerk-limited ("double S" / seven segment) rest-to-rest motion profile for a distance D >= 0.
 * See Biagiotti & Melchiorri, "Trajectory Planning for Automatic Machines and Robots", ch. 3.4.
 */
export interface Limits {
  vMax: number;
  aMax: number;
  jMax: number;
}

export interface ProfileSample {
  s: number;
  v: number;
  a: number;
}

export interface Profile {
  readonly distance: number;
  readonly duration: number;
  sample(t: number): ProfileSample;
}

export function doubleS(distance: number, { vMax, aMax, jMax }: Limits): Profile {
  const D = Math.max(0, distance);
  if (D < 1e-12) return { distance: 0, duration: 0, sample: () => ({ s: 0, v: 0, a: 0 }) };

  let Tj: number; // jerk phase duration
  let Ta: number; // acceleration phase duration (incl. jerk phases)
  let Tv: number; // constant velocity duration

  // Assume vMax is reached
  if (vMax * jMax >= aMax * aMax) {
    Tj = aMax / jMax;
    Ta = Tj + vMax / aMax;
  } else {
    Tj = Math.sqrt(vMax / jMax);
    Ta = 2 * Tj;
  }
  Tv = D / vMax - Ta;

  if (Tv < 0) {
    Tv = 0;
    const triangular = () => {
      Tj = Math.cbrt(D / (2 * jMax));
      Ta = 2 * Tj;
    };
    if (vMax * jMax >= aMax * aMax) {
      // aMax may still be reached: D = aMax (Ta - Tj) Ta
      Tj = aMax / jMax;
      Ta = (Tj + Math.sqrt(Tj * Tj + (4 * D) / aMax)) / 2;
      if (Ta < 2 * Tj) triangular();
    } else {
      triangular();
    }
  }

  const aLim = jMax * Tj;
  const vLim = aLim * (Ta - Tj);
  const sA = (vLim * Ta) / 2; // distance covered while accelerating
  const T = 2 * Ta + Tv;

  const accel = (t: number): ProfileSample => {
    if (t < Tj) return { s: (jMax * t ** 3) / 6, v: (jMax * t * t) / 2, a: jMax * t };
    if (t < Ta - Tj) return { s: (aLim / 6) * (3 * t * t - 3 * Tj * t + Tj * Tj), v: aLim * (t - Tj / 2), a: aLim };
    const r = Ta - t;
    return { s: sA - vLim * r + (jMax * r ** 3) / 6, v: vLim - (jMax * r * r) / 2, a: jMax * r };
  };

  return {
    distance: D,
    duration: T,
    sample(t: number): ProfileSample {
      if (t <= 0) return { s: 0, v: 0, a: 0 };
      if (t >= T) return { s: D, v: 0, a: 0 };
      if (t < Ta) return accel(t);
      if (t <= Ta + Tv) return { s: sA + vLim * (t - Ta), v: vLim, a: 0 };
      const m = accel(T - t); // deceleration mirrors the acceleration phase
      return { s: D - m.s, v: m.v, a: -m.a };
    },
  };
}

/** Scale all limits by a factor k (k < 1 slows the move down). */
export const scaleLimits = (l: Limits, k: number): Limits => ({
  vMax: l.vMax * k,
  aMax: l.aMax * k * k,
  jMax: l.jMax * k * k * k,
});
