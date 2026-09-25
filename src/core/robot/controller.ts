import { add, addScaled, lengthXY, scale, sub, vec3, ZERO, type Vec3 } from '../math/vec3';
import { inverseKinematics, jointVelocities, type Joints } from '../kinematics/kinematics';
import { motorTorques } from '../kinematics/dynamics';
import type { DeltaParams } from '../kinematics/params';
import { planIntercept, smoother, type ArchOptions, type MotionState, type Segment } from '../motion/trajectory';
import {
  nearestYaw,
  orderCandidates,
  planFor,
  reachWindow,
  type Candidate,
  type PlanContext,
  type Strategy,
  type Target,
} from '../scheduling/scheduler';
import { Telemetry } from './telemetry';

/** What a scenario must provide so the controller can work on it. */
export interface TaskProvider {
  /** Items that may be picked (not yet reserved) */
  pickCandidates(): Target[];
  /** Places the given item can go to (e.g. free tray slots, the weed bin) */
  placeCandidates(item: Target): Target[];
  /** Look up a target by id; undefined if it disappeared */
  find(id: number): Target | undefined;
  onPickScheduled(item: Target): void;
  onGrasp(item: Target): void;
  onRelease(item: Target, place: Target): void;
  onAbort(item: Target): void;
  /** Keep-out check (e.g. crops), dt is the time from now */
  clear?(p: Vec3, dt: number): boolean;
  /** Payload mass of an item for the torque estimate */
  payloadMass?(item: Target): number;
}

export interface ControllerConfig {
  params: DeltaParams;
  arch: ArchOptions;
  strategy: Strategy;
  graspTime: number;
  releaseTime: number;
  /** Rest position when there is nothing to do */
  home: Vec3;
  /** Joint limit margin for planning [rad] */
  margin: number;
  maxInterceptTime: number;
}

export type Phase = 'idle' | 'approach' | 'grasp' | 'transfer' | 'release' | 'return';

interface ActiveMove {
  segment: Segment;
  start: number;
  /** For moves towards a live target: where we predicted it to be, for drift correction */
  targetId?: number;
  predicted?: { p: Vec3; v: Vec3 };
}

export interface ControllerStats {
  picks: number;
  places: number;
  cycleTimes: number[];
  busyTime: number;
  totalTime: number;
}

/**
 * Task-level robot controller: a small state machine that asks the scheduler for the next item,
 * intercepts it, tracks it while the gripper closes, carries it to a place target and releases it.
 */
export class RobotController {
  phase: Phase = 'idle';
  state: MotionState;
  theta: Joints = [0, 0, 0];
  omega: Joints = [0, 0, 0];
  torque: Joints = [0, 0, 0];
  /** 0 = open, 1 = closed */
  gripper = 0;
  carrying: Target | null = null;
  pickTarget: Target | null = null;
  placeTarget: Target | null = null;
  /** Planned path of the current move for visualisation, in robot frame */
  preview: Vec3[] = [];
  readonly telemetry = new Telemetry(1200);
  stats: ControllerStats = { picks: 0, places: 0, cycleTimes: [], busyTime: 0, totalTime: 0 };

  private move: ActiveMove | null = null;
  private phaseStart = 0;
  private lastSchedule = -1;
  private cycleStart = 0;
  private lastTelemetry = -1;
  private lastOmega: Joints = [0, 0, 0];
  /** Yaw of the gripper relative to the carried item */
  carryYawOffset = 0;

  constructor(
    public config: ControllerConfig,
    private tasks: TaskProvider,
  ) {
    this.state = { p: config.home, v: ZERO, a: ZERO, yaw: 0 };
    const ik = inverseKinematics(config.params, config.home);
    if (ik.ok) this.theta = ik.theta;
  }

  private get ctx(): PlanContext {
    return {
      params: this.config.params,
      arch: this.config.arch,
      margin: this.config.margin,
      graspTime: this.config.graspTime,
      maxTime: this.config.maxInterceptTime,
      clear: this.tasks.clear?.bind(this.tasks),
    };
  }

  private setPhase(phase: Phase, t: number) {
    this.phase = phase;
    this.phaseStart = t;
  }

  private startMove(segment: Segment, t: number, target?: Target) {
    this.move = {
      segment,
      start: t,
      targetId: target?.id,
      predicted: target ? { p: target.pos, v: target.vel } : undefined,
    };
    this.preview = [];
    const n = 40;
    for (let k = 0; k <= n; k++) this.preview.push(segment.sample((k / n) * segment.duration).p);
  }

  /** Try to find the next pick; returns true if a move was started. */
  private trySchedule(t: number): boolean {
    const ctx = this.ctx;
    const start = { p: this.state.p, v: this.state.v, yaw: this.state.yaw };
    const cands: Candidate[] = [];
    for (const target of this.tasks.pickCandidates()) {
      const window = reachWindow(ctx, target);
      if (window && window[1] > ctx.graspTime) cands.push({ target, window });
    }
    const ordered = orderCandidates(this.config.strategy, cands, this.state.p);
    for (const { target, window } of ordered.slice(0, 8)) {
      const yawTo = nearestYaw(target.yaw, target.symmetry, start.yaw);
      const plan = planFor(ctx, start, target, window, 0, yawTo);
      if (!plan) continue;
      // Make sure the item can be placed afterwards, from the predicted end of the grasp
      const afterGrasp = {
        p: addScaled(plan.meet, target.vel, ctx.graspTime),
        v: target.vel,
        yaw: yawTo,
      };
      const offset = plan.duration + ctx.graspTime;
      if (!this.findPlace(target, afterGrasp, offset, yawTo - target.yaw)) continue;
      this.tasks.onPickScheduled(target);
      this.pickTarget = target;
      this.carryYawOffset = target.symmetry > 0 ? yawTo - target.yaw : 0;
      this.startMove(plan, t, target);
      this.setPhase('approach', t);
      this.cycleStart = t;
      return true;
    }
    return false;
  }

  private findPlace(
    item: Target,
    start: { p: Vec3; v: Vec3; yaw: number },
    offset: number,
    yawOffset: number,
  ): { place: Target; segment: Segment } | null {
    const ctx = { ...this.ctx, graspTime: this.config.releaseTime };
    for (const place of this.tasks.placeCandidates(item)) {
      // window and plan are relative to now; the move itself starts `offset` seconds from now
      const window = reachWindow(ctx, place);
      if (!window) continue;
      const yawTo = nearestYaw(place.yaw + yawOffset, item.symmetry, start.yaw);
      const plan = planFor(ctx, start, place, window, offset, yawTo);
      if (plan) return { place, segment: plan };
    }
    return null;
  }

  private goHome(t: number) {
    const plan = planIntercept(
      { p: this.state.p, v: this.state.v },
      { p: this.config.home, v: ZERO },
      { ...this.config.arch, minLift: 0.02 },
      { yawFrom: this.state.yaw, yawTo: this.state.yaw },
    );
    if (plan) {
      this.startMove(plan, t);
      this.setPhase('return', t);
    } else {
      this.move = null;
      this.state = { ...this.state, v: ZERO, a: ZERO };
      this.setPhase('idle', t);
    }
  }

  private abort(t: number) {
    if (this.pickTarget) this.tasks.onAbort(this.pickTarget);
    if (this.carrying && this.carrying !== this.pickTarget) this.tasks.onAbort(this.carrying);
    this.pickTarget = null;
    this.placeTarget = null;
    this.carrying = null;
    this.gripper = 0;
    this.goHome(t);
  }

  /** Advance the controller to time t (seconds), dt is the step size. */
  step(t: number, dt: number) {
    const cfg = this.config;
    this.stats.totalTime += dt;
    if (this.phase !== 'idle' && this.phase !== 'return') this.stats.busyTime += dt;

    // Look for work when idle or heading home (re-plan at most every 50 ms)
    if ((this.phase === 'idle' || this.phase === 'return') && t - this.lastSchedule >= 0.05) {
      this.lastSchedule = t;
      this.trySchedule(t);
    }

    switch (this.phase) {
      case 'idle':
        this.state = { ...this.state, v: ZERO, a: ZERO };
        break;
      case 'approach':
      case 'transfer':
      case 'return': {
        const done = this.followMove(t);
        if (done) {
          if (this.phase === 'approach') this.setPhase('grasp', t);
          else if (this.phase === 'transfer') this.setPhase('release', t);
          else {
            this.move = null;
            this.state = { ...this.state, v: ZERO, a: ZERO };
            this.setPhase('idle', t);
          }
        }
        break;
      }
      case 'grasp': {
        const target = this.pickTarget && this.tasks.find(this.pickTarget.id);
        if (!target) return this.abort(t);
        this.track(target);
        this.gripper = Math.min(1, (t - this.phaseStart) / cfg.graspTime);
        if (t - this.phaseStart >= cfg.graspTime) {
          this.tasks.onGrasp(target);
          this.carrying = target;
          const start = { p: this.state.p, v: this.state.v, yaw: this.state.yaw };
          const place = this.findPlace(target, start, 0, this.carryYawOffset);
          if (!place) return this.abort(t);
          this.placeTarget = place.place;
          this.startMove(place.segment, t, place.place);
          this.setPhase('transfer', t);
        }
        break;
      }
      case 'release': {
        const place = this.placeTarget && this.tasks.find(this.placeTarget.id);
        if (place) this.track(place);
        else this.state = { ...this.state, p: addScaled(this.state.p, this.state.v, dt), a: ZERO };
        this.gripper = Math.max(0, 1 - (t - this.phaseStart) / cfg.releaseTime);
        if (t - this.phaseStart >= cfg.releaseTime) {
          if (this.carrying && this.placeTarget) this.tasks.onRelease(this.carrying, this.placeTarget);
          this.stats.picks++;
          this.stats.places++;
          this.stats.cycleTimes.push(t - this.cycleStart);
          if (this.stats.cycleTimes.length > 50) this.stats.cycleTimes.shift();
          this.carrying = null;
          this.pickTarget = null;
          this.placeTarget = null;
          this.move = null;
          this.preview = [];
          this.lastSchedule = -1;
          if (!this.trySchedule(t)) this.goHome(t);
        }
        break;
      }
    }

    this.updateJoints(t, dt);
  }

  /** Follow the active move; returns true when it is finished. */
  private followMove(t: number): boolean {
    const m = this.move;
    if (!m) return true;
    const tau = Math.min(t - m.start, m.segment.duration);
    let s = m.segment.sample(tau);
    // Blend in the difference between predicted and actual target motion (e.g. speed changes)
    if (m.targetId !== undefined && m.predicted) {
      const target = this.tasks.find(m.targetId);
      if (!target) {
        this.abort(t);
        return false;
      }
      const predicted = addScaled(m.predicted.p, m.predicted.v, t - m.start);
      const err = sub(target.pos, predicted);
      const w = smoother(tau / Math.max(m.segment.duration, 1e-6)).h;
      s = { ...s, p: add(s.p, scale(vec3(err.x, err.y, 0), w)) };
    }
    this.state = s;
    return t - m.start >= m.segment.duration;
  }

  /** Stay locked onto a moving target. */
  private track(target: Target) {
    this.state = {
      p: vec3(target.pos.x, target.pos.y, target.pos.z),
      v: target.vel,
      a: ZERO,
      yaw: this.state.yaw,
    };
  }

  private updateJoints(t: number, dt: number) {
    const p = this.config.params;
    const ik = inverseKinematics(p, this.state.p, -1);
    if (!ik.ok) return;
    this.theta = ik.theta;
    this.omega = jointVelocities(p, ik.theta, this.state.p, this.state.v);
    const alpha = this.omega.map((w, i) => (w - this.lastOmega[i]!) / dt) as unknown as Joints;
    this.lastOmega = this.omega;
    const payload = this.carrying ? (this.tasks.payloadMass?.(this.carrying) ?? 0) : 0;
    this.torque = motorTorques(p, ik.theta, alpha, this.state.p, this.state.a, payload);
    if (t - this.lastTelemetry >= 0.01) {
      this.lastTelemetry = t;
      const [a, b, c] = this.theta;
      const [wa, wb, wc] = this.omega;
      const [ta, tb, tc] = this.torque;
      this.telemetry.push(t, {
        theta0: a,
        theta1: b,
        theta2: c,
        omega0: wa,
        omega1: wb,
        omega2: wc,
        tau0: ta,
        tau1: tb,
        tau2: tc,
        speed: Math.hypot(lengthXY(this.state.v), this.state.v.z),
      });
    }
  }
}
