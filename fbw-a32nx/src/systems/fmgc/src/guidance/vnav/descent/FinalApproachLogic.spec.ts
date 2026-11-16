// Copyright (c) 2026 FlyByWire Simulations
//
// SPDX-License-Identifier: GPL-3.0

import { describe, it, expect } from 'vitest';
import { ApproachType } from '@flybywiresim/fbw-sdk';
import {
  canFinalAppEngage,
  FinalAppEngageBlocker,
  getFinalAppEngageBlocker,
  FinalAppEngageInputs,
  FinalApproachLegInfo,
  findFinalDescentPointDistanceToEnd,
  isFinalAppApproachType,
} from './FinalApproachLogic';

/**
 * KSAN RNAV (GPS) Y RWY 27, Amdt 4A (FAA Form 8260-3):
 * VYDDA (IF) -2.00- OKAIN -3.50- CIJHI -2.23- REEBO (FAF) -5.17- RW27 (MAP), GPA 3.50°.
 * Distances below are from each leg's termination to the runway.
 */
const KSAN_RNAV_Y_27: FinalApproachLegInfo[] = [
  { distanceToEnd: 12.9 }, // VYDDA
  { distanceToEnd: 10.9 }, // OKAIN
  { distanceToEnd: 7.4 }, // CIJHI
  { distanceToEnd: 5.17 }, // REEBO
  { distanceToEnd: 0, verticalAngle: -3.5 }, // RW27
];

describe('FinalApproachLogic', () => {
  describe('isFinalAppApproachType', () => {
    it('selects FINAL APP for RNAV and navaid approaches', () => {
      for (const type of [
        ApproachType.Rnav,
        ApproachType.Gps,
        ApproachType.Vor,
        ApproachType.VorDme,
        ApproachType.Ndb,
      ]) {
        expect(isFinalAppApproachType(type)).toBe(true);
      }
    });

    it('keeps LOC / G/S for approaches with a localizer-type facility', () => {
      for (const type of [ApproachType.Ils, ApproachType.Loc, ApproachType.Lda, ApproachType.Igs, ApproachType.Sdf]) {
        expect(isFinalAppApproachType(type)).toBe(false);
      }
    });

    it('does not select FINAL APP without an approach', () => {
      expect(isFinalAppApproachType(undefined)).toBe(false);
      expect(isFinalAppApproachType(ApproachType.Unknown)).toBe(false);
    });
  });

  describe('findFinalDescentPointDistanceToEnd', () => {
    it('finds REEBO as the FDP on KSAN RNAV Y 27', () => {
      expect(findFinalDescentPointDistanceToEnd(KSAN_RNAV_Y_27)).toBe(5.17);
    });

    it('uses the start of a chain of coded angles when step-down fixes are on the final path', () => {
      const legs: FinalApproachLegInfo[] = [
        { distanceToEnd: 10 },
        { distanceToEnd: 6.5 }, // FDP
        { distanceToEnd: 3.2, verticalAngle: -3 }, // step-down fix
        { distanceToEnd: 0, verticalAngle: -3 },
      ];

      expect(findFinalDescentPointDistanceToEnd(legs)).toBe(6.5);
    });

    it('returns null when the final leg has no coded angle', () => {
      expect(findFinalDescentPointDistanceToEnd([{ distanceToEnd: 5 }, { distanceToEnd: 0 }])).toBeNull();
      expect(findFinalDescentPointDistanceToEnd([])).toBeNull();
    });

    it('returns null when there is no leg before the coded final path', () => {
      expect(findFinalDescentPointDistanceToEnd([{ distanceToEnd: 0, verticalAngle: -3 }])).toBeNull();
    });
  });

  describe('canFinalAppEngage', () => {
    // Level at 2000 ft approaching REEBO; the 3.5° path passes REEBO at about 2003 ft.
    const atReebo: FinalAppEngageInputs = {
      isApproachPhaseActive: true,
      crossTrackError: 0.05,
      distanceToFinalDescentPoint: 0.2,
      linearDeviation: -3,
      pathAngleAtAircraft: 0,
      finalPathAngle: -3.5,
    };

    it('engages when reaching the FDP on the path', () => {
      expect(canFinalAppEngage(atReebo)).toBe(true);
    });

    it('engages after the FDP when intercepting the descending path from below', () => {
      expect(
        canFinalAppEngage({
          ...atReebo,
          distanceToFinalDescentPoint: -0.5,
          linearDeviation: -40,
          pathAngleAtAircraft: -3.5,
        }),
      ).toBe(true);
    });

    it('does not engage outside the APPROACH phase', () => {
      expect(canFinalAppEngage({ ...atReebo, isApproachPhaseActive: false })).toBe(false);
    });

    it('reports the first blocking condition', () => {
      expect(getFinalAppEngageBlocker(atReebo)).toBe(FinalAppEngageBlocker.None);
      expect(getFinalAppEngageBlocker({ ...atReebo, isApproachPhaseActive: false })).toBe(
        FinalAppEngageBlocker.NotInApproachPhase,
      );
      expect(getFinalAppEngageBlocker({ ...atReebo, crossTrackError: -2 })).toBe(
        FinalAppEngageBlocker.OutsideLateralCaptureArea,
      );
      expect(getFinalAppEngageBlocker({ ...atReebo, distanceToFinalDescentPoint: 3 })).toBe(
        FinalAppEngageBlocker.PathNotDescending,
      );
      expect(getFinalAppEngageBlocker({ ...atReebo, linearDeviation: 300 })).toBe(
        FinalAppEngageBlocker.TooHighAbovePath,
      );
      expect(getFinalAppEngageBlocker({ ...atReebo, linearDeviation: -300 })).toBe(FinalAppEngageBlocker.BelowPath);
    });

    it('does not engage on a level segment more than 1.5 NM before the FDP', () => {
      expect(canFinalAppEngage({ ...atReebo, distanceToFinalDescentPoint: 1.6 })).toBe(false);
      expect(canFinalAppEngage({ ...atReebo, distanceToFinalDescentPoint: 1.5 })).toBe(true);
    });

    it('engages before the FDP when on a descending leg of the profile (APPR pressed early)', () => {
      // Descending from CIJHI (2700) to REEBO (2000), on the path
      expect(
        canFinalAppEngage({
          ...atReebo,
          distanceToFinalDescentPoint: 1.8,
          linearDeviation: 20,
          pathAngleAtAircraft: -2.9,
        }),
      ).toBe(true);
    });

    it('does not engage more than 1.5 NM off the lateral path', () => {
      expect(canFinalAppEngage({ ...atReebo, crossTrackError: 1.6 })).toBe(false);
      expect(canFinalAppEngage({ ...atReebo, crossTrackError: -1.5 })).toBe(true);
    });

    it('does not engage more than 150 ft above the path', () => {
      expect(canFinalAppEngage({ ...atReebo, linearDeviation: 151 })).toBe(false);
      expect(canFinalAppEngage({ ...atReebo, linearDeviation: 150 })).toBe(true);
    });

    it('does not engage before the path has been intercepted from below', () => {
      expect(canFinalAppEngage({ ...atReebo, linearDeviation: -200 })).toBe(false);
    });

    it('does not engage on a level segment after the FDP', () => {
      expect(canFinalAppEngage({ ...atReebo, distanceToFinalDescentPoint: -1, pathAngleAtAircraft: 0 })).toBe(false);
    });
  });
});
