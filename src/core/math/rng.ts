/**
 * Seeded pseudo random number generator (sfc32) with a few distributions.
 * All randomness in the simulation goes through this so runs are reproducible from a seed.
 */
export class Rng {
  private a: number;
  private b: number;
  private c: number;
  private d: number;

  constructor(seed: number) {
    // splitmix32 to spread the seed over the state
    let s = seed >>> 0;
    const next = () => {
      s = (s + 0x9e3779b9) | 0;
      let z = s;
      z = Math.imul(z ^ (z >>> 16), 0x85ebca6b);
      z = Math.imul(z ^ (z >>> 13), 0xc2b2ae35);
      return (z ^ (z >>> 16)) >>> 0;
    };
    this.a = next();
    this.b = next();
    this.c = next();
    this.d = next();
    for (let i = 0; i < 12; i++) this.next();
  }

  /** Uniform in [0, 1). */
  next(): number {
    const t = (((this.a + this.b) | 0) + this.d) | 0;
    this.d = (this.d + 1) | 0;
    this.a = this.b ^ (this.b >>> 9);
    this.b = (this.c + (this.c << 3)) | 0;
    this.c = (this.c << 21) | (this.c >>> 11);
    this.c = (this.c + t) | 0;
    return (t >>> 0) / 4294967296;
  }

  range(min: number, max: number): number {
    return min + (max - min) * this.next();
  }

  int(min: number, maxExclusive: number): number {
    return Math.floor(this.range(min, maxExclusive));
  }

  pick<T>(items: readonly T[]): T {
    const item = items[this.int(0, items.length)];
    if (item === undefined) throw new Error('pick from empty array');
    return item;
  }

  /** Standard normal via Box-Muller. */
  normal(mean = 0, std = 1): number {
    const u = 1 - this.next();
    const v = this.next();
    return mean + std * Math.sqrt(-2 * Math.log(u)) * Math.cos(2 * Math.PI * v);
  }

  /** Exponential inter-arrival time for a Poisson process with the given rate (events per unit). */
  exponential(rate: number): number {
    return -Math.log(1 - this.next()) / rate;
  }

  /** Gamma(shape, 1) via Marsaglia-Tsang, used for the beta distribution. */
  private gamma(shape: number): number {
    if (shape < 1) return this.gamma(shape + 1) * Math.pow(this.next(), 1 / shape);
    const d = shape - 1 / 3;
    const c = 1 / Math.sqrt(9 * d);
    for (;;) {
      const x = this.normal();
      const v = Math.pow(1 + c * x, 3);
      if (v <= 0) continue;
      const u = this.next();
      if (Math.log(u) < 0.5 * x * x + d - d * v + d * Math.log(v)) return d * v;
    }
  }

  beta(alpha: number, beta: number): number {
    const x = this.gamma(alpha);
    const y = this.gamma(beta);
    return x / (x + y);
  }
}

/** Deterministic 32-bit hash for combining a seed with an index (e.g. field chunks). */
export function hashSeed(seed: number, index: number): number {
  let h = (seed ^ Math.imul(index + 0x632be5ab, 0x9e3779b1)) >>> 0;
  h = Math.imul(h ^ (h >>> 15), 0x2c1b3c6d);
  h = Math.imul(h ^ (h >>> 12), 0x297a2d39);
  return (h ^ (h >>> 15)) >>> 0;
}
