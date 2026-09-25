import type { Vec3 } from '../math/vec3';
import type { DeltaParams } from '../kinematics/params';
import type { RobotController } from '../robot/controller';

export type ScenarioId = 'pickPlace' | 'weeding';

export interface Metric {
  label: string;
  value: number;
  unit?: string;
  /** Number of decimals for display */
  digits?: number;
  /** Highlight as good / bad in the UI */
  tone?: 'good' | 'bad' | 'neutral';
}

export type SimEvent =
  | { type: 'picked'; id: number; t: number }
  | { type: 'placed'; id: number; t: number }
  | { type: 'missed'; id: number; t: number }
  | { type: 'cropHit'; id: number; t: number };

/** Common interface of all scenarios so the UI, renderer and benchmark can treat them alike. */
export interface Scenario {
  readonly id: ScenarioId;
  readonly robot: RobotController;
  readonly params: DeltaParams;
  /** Current simulation time [s] */
  readonly time: number;
  /** World position of the robot base (the robot frame is the world frame translated to this point) */
  robotBase(): Vec3;
  step(dt: number): void;
  metrics(): Metric[];
  /** Set to true when a limited run has finished */
  readonly finished: boolean;
}
