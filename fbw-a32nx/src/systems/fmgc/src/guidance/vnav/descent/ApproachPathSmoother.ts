// Copyright (c) 2026 FlyByWire Simulations
//
// SPDX-License-Identifier: GPL-3.0

/**
 * Reshapes the approach part of the descent profile so that it can be flown in FINAL APP without overshooting:
 *
 * 1. Level segments in the middle of a descent (level decelerations planned at altitude constraints) are removed. The
 *    path crosses each constraint inside such a segment exactly at the constraint altitude and descends in straight
 *    lines between constraints. A level segment spanning two constraints (a coded level segment) is kept between them.
 * 2. Corners where the path angle changes are rounded with a few short segments, sized from the speed and a vertical
 *    acceleration limit. Level-offs (the path becoming shallower) use a gentler limit, so they are spread over a
 *    longer distance. A corner at an altitude constraint never goes below the constraint: steepening corners at a
 *    constraint start the blend at the constraint instead of before it.
 *
 * If the result would go below an altitude constraint that the original path respected, the corner rounding is dropped
 * and, if needed, the whole reshaping.
 */

export interface SmoothingPoint {
  distanceFromStart: NauticalMiles;
  altitude: Feet;
}

export interface SmoothingConstraint {
  distanceFromStart: NauticalMiles;
  minimumAltitude?: Feet;
  maximumAltitude?: Feet;
}

/** Altitude difference below which consecutive points are considered level. */
const LEVEL_TOLERANCE: Feet = 5;
/** How much the path must descend before and after a level segment for it to count as being in the middle of a descent. */
const DESCENT_TOLERANCE: Feet = 20;
/** How far outside a level segment a constraint may lie and still be considered to be in it. */
const CONSTRAINT_DISTANCE_TOLERANCE: NauticalMiles = 0.1;
/** Gradient changes smaller than this are left alone. 40 ft/NM is about 0.4°, or 150 ft/min at 220 kt. */
const MIN_GRADIENT_CHANGE: number = 40;
/** Vertical acceleration used to size the rounding when the path becomes steeper. */
const STEEPENING_VERTICAL_ACCELERATION_G = 0.08;
/** Vertical acceleration used to size the rounding when the path becomes shallower (levelling off). */
const LEVELLING_VERTICAL_ACCELERATION_G = 0.035;
/** Number of segments a corner is replaced with. */
const BLEND_SEGMENTS = 4;
/** Fraction of an adjacent segment a corner may use up. */
const MAX_SEGMENT_FRACTION = 0.45;
const MAX_BLEND_LENGTH: NauticalMiles = 2;
const DISTANCE_EPSILON: NauticalMiles = 1e-4;
const CONSTRAINT_ALTITUDE_TOLERANCE: Feet = 5;
/** How far above an "at" or "at or below" constraint the rounding of a level-off may pass. */
const MAX_ROUNDING_EXCESS_AT_CONSTRAINT: Feet = 50;

const FEET_PER_MINUTE_PER_G_PER_SECOND = 32.174 * 60;

/**
 * @param path the vertical path, sorted by distance
 * @param constraints the descent altitude constraints
 * @param startDistance only level segments starting at or after this distance, and corners at or after it, are changed
 * @param speedAtDistance the predicted speed (kt) along the path, used to size the corner rounding
 * @returns the reshaped path, or the original path if nothing was changed
 */
export function smoothApproachPath(
  path: readonly SmoothingPoint[],
  constraints: readonly SmoothingConstraint[],
  startDistance: NauticalMiles,
  speedAtDistance: (distanceFromStart: NauticalMiles) => Knots,
): SmoothingPoint[] {
  const deduplicated = removeDuplicateDistances(path);
  const withoutLevelSegments = removeIntermediateLevelSegments(deduplicated, constraints, startDistance);
  const rounded = roundCorners(withoutLevelSegments, constraints, startDistance, speedAtDistance);

  if (respectsConstraints(rounded, deduplicated, constraints, startDistance)) {
    return rounded;
  }

  if (respectsConstraints(withoutLevelSegments, deduplicated, constraints, startDistance)) {
    return withoutLevelSegments;
  }

  return deduplicated;
}

export function interpolatePathAltitude(path: readonly SmoothingPoint[], distanceFromStart: NauticalMiles): Feet {
  if (path.length === 0) {
    return 0;
  }

  if (distanceFromStart <= path[0].distanceFromStart) {
    return path[0].altitude;
  }

  for (let i = 1; i < path.length; i++) {
    const end = path[i];

    if (distanceFromStart <= end.distanceFromStart) {
      const start = path[i - 1];
      const length = end.distanceFromStart - start.distanceFromStart;

      if (length < DISTANCE_EPSILON) {
        return end.altitude;
      }

      return (
        start.altitude + ((end.altitude - start.altitude) * (distanceFromStart - start.distanceFromStart)) / length
      );
    }
  }

  return path[path.length - 1].altitude;
}

function removeDuplicateDistances(path: readonly SmoothingPoint[]): SmoothingPoint[] {
  const result: SmoothingPoint[] = [];

  for (const point of path) {
    const last = result[result.length - 1];

    if (last && point.distanceFromStart - last.distanceFromStart < DISTANCE_EPSILON) {
      result[result.length - 1] = { distanceFromStart: point.distanceFromStart, altitude: point.altitude };
    } else {
      result.push({ distanceFromStart: point.distanceFromStart, altitude: point.altitude });
    }
  }

  return result;
}

function removeIntermediateLevelSegments(
  path: readonly SmoothingPoint[],
  constraints: readonly SmoothingConstraint[],
  startDistance: NauticalMiles,
): SmoothingPoint[] {
  const result: SmoothingPoint[] = [];

  let i = 0;
  while (i < path.length) {
    const levelStart = path[i];

    // Find the end of a level run starting at i
    let j = i;
    while (j + 1 < path.length && Math.abs(path[j + 1].altitude - levelStart.altitude) < LEVEL_TOLERANCE) {
      j++;
    }

    const levelEnd = path[j];
    const before = result[result.length - 1];
    const after = path[j + 1];

    const isIntermediateLevelSegment =
      j > i &&
      levelStart.distanceFromStart >= startDistance &&
      before !== undefined &&
      after !== undefined &&
      before.altitude > levelStart.altitude + DESCENT_TOLERANCE &&
      after.altitude < levelStart.altitude - DESCENT_TOLERANCE;

    if (!isIntermediateLevelSegment) {
      for (let k = i; k <= j; k++) {
        result.push(path[k]);
      }
      i = j + 1;
      continue;
    }

    let firstAnchor = Infinity;
    let lastAnchor = -Infinity;
    for (const constraint of constraints) {
      if (
        constraint.distanceFromStart >= levelStart.distanceFromStart - CONSTRAINT_DISTANCE_TOLERANCE &&
        constraint.distanceFromStart <= levelEnd.distanceFromStart + CONSTRAINT_DISTANCE_TOLERANCE
      ) {
        firstAnchor = Math.min(firstAnchor, constraint.distanceFromStart);
        lastAnchor = Math.max(lastAnchor, constraint.distanceFromStart);
      }
    }

    if (!Number.isFinite(firstAnchor)) {
      // No constraint: the level segment only exists to decelerate. Keep its middle as the crossing point.
      firstAnchor = (levelStart.distanceFromStart + levelEnd.distanceFromStart) / 2;
      lastAnchor = firstAnchor;
    }

    firstAnchor = clamp(firstAnchor, levelStart.distanceFromStart, levelEnd.distanceFromStart);
    lastAnchor = clamp(lastAnchor, levelStart.distanceFromStart, levelEnd.distanceFromStart);

    result.push({ distanceFromStart: firstAnchor, altitude: levelStart.altitude });
    if (lastAnchor - firstAnchor > DISTANCE_EPSILON) {
      result.push({ distanceFromStart: lastAnchor, altitude: levelStart.altitude });
    }

    i = j + 1;
  }

  return result;
}

function roundCorners(
  path: readonly SmoothingPoint[],
  constraints: readonly SmoothingConstraint[],
  startDistance: NauticalMiles,
  speedAtDistance: (distanceFromStart: NauticalMiles) => Knots,
): SmoothingPoint[] {
  if (path.length < 3) {
    return path.slice();
  }

  const result: SmoothingPoint[] = [path[0]];

  for (let i = 1; i < path.length - 1; i++) {
    const previous = result[result.length - 1];
    const corner = path[i];
    const next = path[i + 1];

    const incomingLength = corner.distanceFromStart - previous.distanceFromStart;
    const outgoingLength = next.distanceFromStart - corner.distanceFromStart;

    if (
      corner.distanceFromStart < startDistance ||
      incomingLength < DISTANCE_EPSILON ||
      outgoingLength < DISTANCE_EPSILON
    ) {
      result.push(corner);
      continue;
    }

    // Gradients in ft/NM, positive when descending
    const incomingGradient = (previous.altitude - corner.altitude) / incomingLength;
    const outgoingGradient = (corner.altitude - next.altitude) / outgoingLength;
    const gradientChange = outgoingGradient - incomingGradient;

    if (Math.abs(gradientChange) < MIN_GRADIENT_CHANGE) {
      result.push(corner);
      continue;
    }

    const isSteepening = gradientChange > 0;
    const constraint = findConstraintAt(constraints, corner.distanceFromStart);

    const trueAirspeed = approximateTrueAirspeed(speedAtDistance(corner.distanceFromStart), corner.altitude);
    const verticalAcceleration =
      (isSteepening ? STEEPENING_VERTICAL_ACCELERATION_G : LEVELLING_VERTICAL_ACCELERATION_G) *
      FEET_PER_MINUTE_PER_G_PER_SECOND;
    // Time to change the vertical speed by the gradient change at this speed, times the distance covered per second.
    const verticalSpeedChange = (Math.abs(gradientChange) * trueAirspeed) / 60;
    const desiredLength = ((verticalSpeedChange / verticalAcceleration) * trueAirspeed) / 3600;

    if (isSteepening && constraint?.minimumAltitude !== undefined) {
      // Steepening at a constraint: start the rounding at the constraint so the path never goes below it. The path
      // ends up slightly above the outgoing segment and rejoins it at the next point.
      const length = Math.min(desiredLength, MAX_BLEND_LENGTH, MAX_SEGMENT_FRACTION * outgoingLength);

      result.push(corner);
      for (let k = 1; k <= BLEND_SEGMENTS; k++) {
        const x = (length * k) / BLEND_SEGMENTS;
        // Gradient increases linearly from the incoming to the outgoing gradient over the rounding
        const altitudeLost = incomingGradient * x + (gradientChange * x * x) / (2 * length);
        result.push({ distanceFromStart: corner.distanceFromStart + x, altitude: corner.altitude - altitudeLost });
      }
      continue;
    }

    if (!isSteepening && constraint?.maximumAltitude !== undefined && constraint.minimumAltitude === undefined) {
      // Levelling off at an at-or-below constraint: rounding would cross above it
      result.push(corner);
      continue;
    }

    // Symmetric rounding (quadratic Bézier with the corner as control point). It passes above the corner when levelling
    // off and below it when steepening away from a constraint.
    let halfLength = Math.min(
      desiredLength / 2,
      MAX_BLEND_LENGTH / 2,
      MAX_SEGMENT_FRACTION * incomingLength,
      MAX_SEGMENT_FRACTION * outgoingLength,
    );
    if (constraint?.maximumAltitude !== undefined) {
      // The rounding passes the corner |change| * halfLength / 4 above it
      halfLength = Math.min(halfLength, (4 * MAX_ROUNDING_EXCESS_AT_CONSTRAINT) / Math.abs(gradientChange));
    }
    const startAltitude = corner.altitude + incomingGradient * halfLength;
    const endAltitude = corner.altitude - outgoingGradient * halfLength;

    for (let k = 0; k <= BLEND_SEGMENTS; k++) {
      const t = k / BLEND_SEGMENTS;
      const altitude = (1 - t) * (1 - t) * startAltitude + 2 * (1 - t) * t * corner.altitude + t * t * endAltitude;
      result.push({ distanceFromStart: corner.distanceFromStart - halfLength + 2 * halfLength * t, altitude });
    }
  }

  result.push(path[path.length - 1]);

  return result;
}

function respectsConstraints(
  path: readonly SmoothingPoint[],
  original: readonly SmoothingPoint[],
  constraints: readonly SmoothingConstraint[],
  startDistance: NauticalMiles,
): boolean {
  for (const constraint of constraints) {
    if (constraint.distanceFromStart < startDistance) {
      continue;
    }

    const altitude = interpolatePathAltitude(path, constraint.distanceFromStart);
    const originalAltitude = interpolatePathAltitude(original, constraint.distanceFromStart);

    if (
      constraint.minimumAltitude !== undefined &&
      altitude < Math.min(constraint.minimumAltitude, originalAltitude) - CONSTRAINT_ALTITUDE_TOLERANCE
    ) {
      return false;
    }

    if (
      constraint.maximumAltitude !== undefined &&
      altitude >
        Math.max(constraint.maximumAltitude, originalAltitude) +
          MAX_ROUNDING_EXCESS_AT_CONSTRAINT +
          CONSTRAINT_ALTITUDE_TOLERANCE
    ) {
      return false;
    }
  }

  return true;
}

function findConstraintAt(
  constraints: readonly SmoothingConstraint[],
  distanceFromStart: NauticalMiles,
): SmoothingConstraint | undefined {
  for (const constraint of constraints) {
    if (Math.abs(constraint.distanceFromStart - distanceFromStart) <= CONSTRAINT_DISTANCE_TOLERANCE) {
      return constraint;
    }
  }

  return undefined;
}

function approximateTrueAirspeed(indicatedAirspeed: Knots, altitude: Feet): Knots {
  // About 2% per 1000 ft
  return indicatedAirspeed * (1 + Math.max(0, altitude) / 50_000);
}

function clamp(value: number, min: number, max: number): number {
  return Math.min(Math.max(value, min), max);
}
