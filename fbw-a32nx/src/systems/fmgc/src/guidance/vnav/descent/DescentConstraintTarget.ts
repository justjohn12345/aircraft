// Copyright (c) 2026 FlyByWire Simulations
//
// SPDX-License-Identifier: GPL-3.0

/** Tolerance between the aircraft (or FCU) altitude and a constraint altitude. */
export const DESCENT_CONSTRAINT_ALTITUDE_TOLERANCE: Feet = 20;

/**
 * Steepest descent gradient assumed possible when deciding that a constraint is certainly crossed above, roughly an
 * idle descent with speedbrakes extended.
 */
export const DESCENT_CONSTRAINT_MAX_GRADIENT: number = 500; // ft/NM

/** Factor applied to the present descent gradient for the steepest gradient assumed possible. */
export const DESCENT_CONSTRAINT_GRADIENT_FACTOR = 1.5;

/** Below this rate of descent, the present descent gradient is not taken into account. */
export const DESCENT_CONSTRAINT_MIN_VS: FeetPerMinute = 200;

/** A descent altitude constraint, reduced to the lower altitude limit it imposes. */
export interface DescentTargetConstraint {
  distanceFromStart: NauticalMiles;
  /** Lowest altitude allowed at the constrained waypoint. */
  minimumAltitude: Feet;
  /** Whether the constraint was found to be certainly crossed above at the previous evaluation. */
  wasReleased: boolean;
}

export interface DescentConstraintTargetResult<T extends DescentTargetConstraint> {
  /** The altitude of the constraint to use as target, or null if the FCU altitude is the target. */
  altitude: Feet | null;
  /** The constraints ahead of the target that are certainly crossed above. */
  released: T[];
}

/**
 * Selects the descent altitude constraint the FMS sends to the FG as altitude target: the next constraint ahead above
 * the FCU altitude, unless the aircraft certainly crosses it above its altitude, in which case no level-off happens
 * there and the following constraint is used.
 *
 * A constraint is certainly crossed above when the altitude still to lose before it is more than the aircraft can lose
 * over the remaining distance, even at the steeper of {@link DESCENT_CONSTRAINT_MAX_GRADIENT} and
 * {@link DESCENT_CONSTRAINT_GRADIENT_FACTOR} times the present gradient. Once released, a constraint only becomes the
 * target again if the aircraft would reach its altitude before the waypoint at the present gradient, so that a steeper
 * descent alone, e.g. when DES pitches down to regain the path, does not move the target back up.
 * @param presentDistanceFromStart distance of the aircraft along the path
 * @param altitude present altitude
 * @param verticalSpeed present vertical speed, negative in descent, or null if unknown
 * @param groundSpeed present ground speed
 * @param constraints descent altitude constraints not sequenced yet, ordered by distance
 * @param fcuAltitude FCU selected altitude
 * @returns the target, and the constraints released on the way to it, to be passed back as `wasReleased`
 */
export function findDescentConstraintTarget<T extends DescentTargetConstraint>(
  presentDistanceFromStart: NauticalMiles,
  altitude: Feet,
  verticalSpeed: FeetPerMinute | null,
  groundSpeed: Knots,
  constraints: readonly T[],
  fcuAltitude: Feet,
): DescentConstraintTargetResult<T> {
  const presentGradient =
    verticalSpeed !== null && verticalSpeed < -DESCENT_CONSTRAINT_MIN_VS && groundSpeed > 0
      ? -verticalSpeed / (groundSpeed / 60)
      : 0;
  const maxGradient = Math.max(DESCENT_CONSTRAINT_MAX_GRADIENT, DESCENT_CONSTRAINT_GRADIENT_FACTOR * presentGradient);

  const released: T[] = [];

  for (const constraint of constraints) {
    if (
      constraint.distanceFromStart <= presentDistanceFromStart ||
      constraint.minimumAltitude <= fcuAltitude + DESCENT_CONSTRAINT_ALTITUDE_TOLERANCE ||
      // Already at or below the constraint: it cannot be captured in descent (holding it is handled by the caller)
      altitude <= constraint.minimumAltitude + DESCENT_CONSTRAINT_ALTITUDE_TOLERANCE
    ) {
      continue;
    }

    const distanceToGo = constraint.distanceFromStart - presentDistanceFromStart;
    const releaseGradient = constraint.wasReleased ? presentGradient : maxGradient;
    const isCertainlyCrossedAbove = altitude - constraint.minimumAltitude > distanceToGo * releaseGradient;

    if (!isCertainlyCrossedAbove) {
      return { altitude: constraint.minimumAltitude, released };
    }

    released.push(constraint);
  }

  return { altitude: null, released };
}
