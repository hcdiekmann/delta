const deg = (d: number) => (d * Math.PI) / 180;

/**
 * Geometry and mass properties of a delta robot.
 *
 * Frame: origin at the centre of the base plate, Z up, the effector hangs below (negative z).
 * Leg i is mounted at azimuth i * 120deg. Joint angle convention: theta = 0 means the upper arm is
 * horizontal and points radially outward, positive theta raises the elbow.
 */
export interface DeltaParams {
  /** Upper arm length U (motor axis to elbow) [m] */
  upperArm: number;
  /** Lower arm (parallelogram rod) length L [m] */
  lowerArm: number;
  /** Radius from base centre to the motor axes [m] */
  baseRadius: number;
  /** Radius from effector centre to the lower arm joints [m] */
  effectorRadius: number;
  /** Joint limits [rad], in the up-positive convention above */
  thetaMin: number;
  thetaMax: number;
  /**
   * Maximum out-of-plane angle of the lower arm (angle between the rod and the plane
   * perpendicular to the motor axis) before the ball joints dislocate [rad].
   */
  ballJointMax: number;
  /** Mass properties used for the torque estimate */
  upperArmMass: number;
  lowerArmMass: number;
  effectorMass: number;
  gearRatio: number;
  /** Motor + gearbox inertia as seen from the motor shaft [kg m^2] */
  motorInertia: number;
}

/**
 * Joint limits of the original machine: -50..+100deg measured downwards, plus the offset of the
 * 347 x 27.5 mm upper arm link. Converted here to the up-positive convention.
 */
const LINK_OFFSET = Math.atan(27.5 / 347);
const LEGACY_LIMITS = {
  thetaMin: -(deg(100) + LINK_OFFSET),
  thetaMax: deg(50) - LINK_OFFSET,
  // legacy: rod may not tilt past 43.42deg towards the motor axis -> 46.58deg out of plane
  ballJointMax: deg(90 - 43.42),
};

export const PRESETS = {
  picker: {
    upperArm: 0.6,
    lowerArm: 1.0,
    baseRadius: 0.2,
    effectorRadius: 0.045,
    ...LEGACY_LIMITS,
    upperArmMass: 1.043,
    lowerArmMass: 0.186,
    effectorMass: 3.262,
    gearRatio: 38.5,
    motorInertia: 0.000144,
  },
  weeder: {
    upperArm: 0.4,
    lowerArm: 0.65,
    baseRadius: 0.2,
    effectorRadius: 0.045,
    ...LEGACY_LIMITS,
    upperArmMass: 0.6,
    lowerArmMass: 0.12,
    effectorMass: 1.5,
    gearRatio: 38.5,
    motorInertia: 0.000144,
  },
} as const satisfies Record<string, DeltaParams>;

export type PresetName = keyof typeof PRESETS;

export const LEG_ANGLES = [0, (2 * Math.PI) / 3, (4 * Math.PI) / 3] as const;
export type LegIndex = 0 | 1 | 2;
export const LEGS: readonly LegIndex[] = [0, 1, 2];
