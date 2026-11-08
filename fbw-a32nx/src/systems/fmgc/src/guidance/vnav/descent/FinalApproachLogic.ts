// Copyright (c) 2026 FlyByWire Simulations
//
// SPDX-License-Identifier: GPL-3.0

import { ApproachType } from '@flybywiresim/fbw-sdk';

/**
 * FINAL APP capture area. Source: A320 FCOM DSC-22_30-80-30-20 (FINAL APP engagement conditions: "within the capture
 * area of the FMS profile: laterally ±1.5 NM, vertically +150 ft", and "intercepts a descending leg of the vertical
 * flight path"). The lateral limit is applied to the cross-track error. The FCOM notes that FINAL APP may engage before
 * the FDP if APPR is pressed early, so engagement is not restricted to the vicinity of the FDP.
 */
export const FINAL_APP_CAPTURE_MAX_CROSS_TRACK_ERROR: NauticalMiles = 1.5;
export const FINAL_APP_CAPTURE_MAX_ABOVE_PATH: Feet = 150;

/**
 * Distance before the FDP from which a level segment leading to the FDP counts as intercepting the final descending
 * leg, so that FINAL APP is engaged when the final descent begins rather than slightly after it.
 */
export const FINAL_APP_FDP_ANTICIPATION_DISTANCE: NauticalMiles = 1.5;

/**
 * How far below the vertical path the aircraft may be while still being considered to intercept it.
 * The FCOM only states that the aircraft must intercept a descending leg of the vertical path; this tolerance
 * stands in for the moment of interception.
 */
export const FINAL_APP_CAPTURE_MAX_BELOW_PATH: Feet = 50;

/**
 * Whether the given approach type is flown with APP NAV / FINAL (non-LS approach) rather than LOC / G/S.
 * Approaches using a localizer-type facility keep the LS behaviour.
 */
export function isFinalAppApproachType(approachType: ApproachType | undefined): boolean {
  switch (approachType) {
    case ApproachType.Gps:
    case ApproachType.Rnav:
    case ApproachType.Fms:
    case ApproachType.Vor:
    case ApproachType.VorDme:
    case ApproachType.Vortac:
    case ApproachType.Tacan:
    case ApproachType.Ndb:
    case ApproachType.NdbDme:
      return true;
    default:
      return false;
  }
}

/** A leg of the final approach, in flight plan order, used to locate the Final Descent Point. */
export interface FinalApproachLegInfo {
  /** Coded vertical path angle of the leg in degrees (negative = descending), if any. */
  verticalAngle?: Degrees;
  /** Distance from the termination of this leg to the end of the descent profile. */
  distanceToEnd: NauticalMiles;
}

/**
 * Finds the Final Descent Point: the start of the final, continuous run of legs with a coded vertical angle that
 * ends at the destination leg (i.e. the termination of the last leg before that run).
 * @param legs legs from the start of the plan up to and including the destination leg, in order
 * @returns the distance from the FDP to the end of the profile, or null if the final leg has no coded vertical angle
 */
export function findFinalDescentPointDistanceToEnd(legs: readonly FinalApproachLegInfo[]): NauticalMiles | null {
  let i = legs.length - 1;

  if (i < 0 || !isDescendingAngle(legs[i].verticalAngle)) {
    return null;
  }

  while (i > 0 && isDescendingAngle(legs[i - 1].verticalAngle)) {
    i--;
  }

  // The FDP is the termination of the leg before the first leg with a coded angle.
  return i > 0 ? legs[i - 1].distanceToEnd : null;
}

function isDescendingAngle(angle: Degrees | undefined): boolean {
  return angle !== undefined && Number.isFinite(angle) && angle < 0;
}

export interface FinalAppEngageInputs {
  /**
   * Whether the FMS APPROACH phase is active. The FCOM also requires the decel point to be sequenced; the APPROACH
   * phase is activated when it is sequenced (or manually by the crew), so it is not checked separately.
   */
  isApproachPhaseActive: boolean;
  /** Lateral cross-track error from the active leg. */
  crossTrackError: NauticalMiles;
  /** Along-track distance from the aircraft to the FDP; positive before the FDP, negative after it. */
  distanceToFinalDescentPoint: NauticalMiles;
  /** Aircraft altitude minus the vertical path altitude; positive when above the path. */
  linearDeviation: Feet;
  /** Vertical path angle at the aircraft position (negative = descending). */
  pathAngleAtAircraft: Degrees;
  /** Vertical path angle immediately after the FDP (negative = descending). */
  finalPathAngle: Degrees;
}

/** The first FINAL APP engagement condition that is not met, or None if FINAL APP may engage. */
export enum FinalAppEngageBlocker {
  None = 0,
  NotInApproachPhase = 1,
  OutsideLateralCaptureArea = 2,
  TooHighAbovePath = 3,
  BelowPath = 4,
  PathNotDescending = 5,
  // Values set by the caller, not by getFinalAppEngageBlocker
  InvalidProfile = 10,
  NoFinalDescentPoint = 11,
}

/**
 * Returns the first FMS condition preventing FINAL APP engagement, or None.
 * NAV/APP NAV engaged and FINAL armed are checked by the FMGC itself.
 * Source: A320 FCOM DSC-22_30-80-30-20.
 */
export function getFinalAppEngageBlocker(inputs: FinalAppEngageInputs): FinalAppEngageBlocker {
  if (!inputs.isApproachPhaseActive) {
    return FinalAppEngageBlocker.NotInApproachPhase;
  }

  // Capture area: laterally within 1.5 NM of the path
  if (Math.abs(inputs.crossTrackError) > FINAL_APP_CAPTURE_MAX_CROSS_TRACK_ERROR) {
    return FinalAppEngageBlocker.OutsideLateralCaptureArea;
  }

  // Capture area: no more than 150 ft above the path
  if (inputs.linearDeviation > FINAL_APP_CAPTURE_MAX_ABOVE_PATH) {
    return FinalAppEngageBlocker.TooHighAbovePath;
  }

  // The aircraft must intercept a descending leg of the vertical path
  if (inputs.linearDeviation < -FINAL_APP_CAPTURE_MAX_BELOW_PATH) {
    return FinalAppEngageBlocker.BelowPath;
  }

  const isOnDescendingLeg = inputs.pathAngleAtAircraft < 0;
  const isApproachingFinalDescent =
    inputs.distanceToFinalDescentPoint > 0 &&
    inputs.distanceToFinalDescentPoint <= FINAL_APP_FDP_ANTICIPATION_DISTANCE &&
    inputs.finalPathAngle < 0;

  return isOnDescendingLeg || isApproachingFinalDescent
    ? FinalAppEngageBlocker.None
    : FinalAppEngageBlocker.PathNotDescending;
}

/** Whether the FMS allows FINAL APP to engage. See {@link getFinalAppEngageBlocker}. */
export function canFinalAppEngage(inputs: FinalAppEngageInputs): boolean {
  return getFinalAppEngageBlocker(inputs) === FinalAppEngageBlocker.None;
}
