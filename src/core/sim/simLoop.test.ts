import { describe, expect, it } from 'vitest';
import { SIM_DT, SimLoop } from './simLoop';

describe('SimLoop', () => {
  it('takes fixed steps and carries the remainder', () => {
    let n = 0;
    const loop = new SimLoop(() => n++);
    loop.advance(SIM_DT * 2.5);
    expect(n).toBe(2);
    loop.advance(SIM_DT * 0.6);
    expect(n).toBe(3);
  });

  it('applies the time scale and pause', () => {
    let n = 0;
    const loop = new SimLoop(() => n++);
    loop.timeScale = 4;
    loop.advance(SIM_DT * 1.01);
    expect(n).toBe(4);
    loop.paused = true;
    loop.advance(1);
    expect(n).toBe(4);
  });

  it('caps the work per frame', () => {
    let n = 0;
    const loop = new SimLoop(() => n++);
    loop.timeScale = 8;
    loop.advance(0.1);
    expect(n).toBe(64);
  });
});
