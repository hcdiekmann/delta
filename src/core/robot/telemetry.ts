/** Fixed-size ring buffers of recent robot signals for the live charts. */
export const CHANNELS = [
  'theta0',
  'theta1',
  'theta2',
  'omega0',
  'omega1',
  'omega2',
  'tau0',
  'tau1',
  'tau2',
  'speed',
] as const;
export type Channel = (typeof CHANNELS)[number];

export class Telemetry {
  readonly time: Float64Array;
  readonly data: Record<Channel, Float32Array>;
  private head = 0;
  private count = 0;

  constructor(readonly capacity = 1000) {
    this.time = new Float64Array(capacity);
    this.data = Object.fromEntries(CHANNELS.map((c) => [c, new Float32Array(capacity)])) as Record<
      Channel,
      Float32Array
    >;
  }

  push(t: number, values: Record<Channel, number>) {
    this.time[this.head] = t;
    for (const c of CHANNELS) this.data[c][this.head] = values[c];
    this.head = (this.head + 1) % this.capacity;
    this.count = Math.min(this.count + 1, this.capacity);
  }

  clear() {
    this.head = 0;
    this.count = 0;
  }

  get length() {
    return this.count;
  }

  /** Copy out the buffered samples in chronological order. */
  snapshot(channels: readonly Channel[]): { t: number[]; series: number[][] } {
    const start = (this.head - this.count + this.capacity) % this.capacity;
    const t: number[] = new Array(this.count);
    const series = channels.map(() => new Array<number>(this.count));
    for (let k = 0; k < this.count; k++) {
      const idx = (start + k) % this.capacity;
      t[k] = this.time[idx]!;
      channels.forEach((c, j) => (series[j]![k] = this.data[c][idx]!));
    }
    return { t, series };
  }
}
