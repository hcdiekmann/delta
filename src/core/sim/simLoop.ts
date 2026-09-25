export const SIM_DT = 1 / 240;
const MAX_STEPS_PER_FRAME = 64;

/**
 * Fixed time step driver: the simulation always advances in SIM_DT steps regardless of the render
 * frame rate, which keeps it deterministic. Real time is scaled by `timeScale`.
 */
export class SimLoop {
  timeScale = 1;
  paused = false;
  private accumulator = 0;

  constructor(private readonly stepFn: (dt: number) => void) {}

  /** Advance by a real (wall clock) frame duration; returns the number of steps taken. */
  advance(frameSeconds: number): number {
    if (this.paused) return 0;
    this.accumulator += Math.min(frameSeconds, 0.1) * this.timeScale;
    let steps = 0;
    while (this.accumulator >= SIM_DT && steps < MAX_STEPS_PER_FRAME) {
      this.stepFn(SIM_DT);
      this.accumulator -= SIM_DT;
      steps++;
    }
    // if we cannot keep up, drop the backlog instead of spiralling
    if (steps === MAX_STEPS_PER_FRAME) this.accumulator = 0;
    return steps;
  }

  /** Single step while paused. */
  stepOnce() {
    this.stepFn(SIM_DT);
  }
}

/** Run a scenario-like step function for a given simulated duration (tests, benchmark). */
export function runFor(step: (dt: number) => void, seconds: number) {
  const n = Math.round(seconds / SIM_DT);
  for (let i = 0; i < n; i++) step(SIM_DT);
}
