// Copyright (c) 2026 FlyByWire Simulations
//
// SPDX-License-Identifier: GPL-3.0

import { describe, it, expect } from 'vitest';
import { DescentTargetConstraint, findDescentConstraintTarget as find } from './DescentConstraintTarget';

/** Returns only the target altitude, for constraints never released before. */
const findDescentConstraintTarget = (...args: Parameters<typeof find>) => find(...args).altitude;

/** Simplified LUCKI1 arrival: BARET at or above 8000 ft, LUCKI at or above 6000 ft, VYDDA at or above 4000 ft. */
const BARET: DescentTargetConstraint = { distanceFromStart: 110, minimumAltitude: 8000, wasReleased: false };
const LUCKI: DescentTargetConstraint = { distanceFromStart: 125, minimumAltitude: 6000, wasReleased: false };
const VYDDA: DescentTargetConstraint = { distanceFromStart: 140, minimumAltitude: 4000, wasReleased: false };
const CONSTRAINTS = [BARET, LUCKI, VYDDA];
const FCU_ALTITUDE = 2000;

// 1500 ft/min at 300 kt: 300 ft/NM
const VS = -1500;
const GS = 300;

describe('findDescentConstraintTarget', () => {
  it('targets the next constraint while it can still be reached', () => {
    // 25 NM before BARET at FL200: 12000 ft to lose, up to 12500 ft can be lost
    expect(findDescentConstraintTarget(85, 20000, VS, GS, CONSTRAINTS, FCU_ALTITUDE)).toBe(8000);
    // 20 NM before BARET: up to 10000 ft can be lost, so BARET is crossed above and LUCKI is next
    expect(findDescentConstraintTarget(90, 20000, VS, GS, CONSTRAINTS, FCU_ALTITUDE)).toBe(6000);
  });

  it('moves on to the following constraint once the next one is certainly crossed above', () => {
    // 2 NM before BARET at 9500 ft: 1500 ft to lose, at most 1000 ft can be lost
    expect(findDescentConstraintTarget(108, 9500, VS, GS, CONSTRAINTS, FCU_ALTITUDE)).toBe(6000);
  });

  it('keeps the target while closing in on a constraint crossed above, so it does not move back up', () => {
    // Descending at 300 ft/NM towards BARET, crossing it 900 ft above: certainly crossed above within 4.5 NM
    for (let distance = 100; distance < 110; distance += 0.25) {
      const altitude = 8900 + (110 - distance) * 300;
      const target = findDescentConstraintTarget(distance, altitude, VS, GS, CONSTRAINTS, FCU_ALTITUDE);
      if (distance > 105.5) {
        expect(target).toBe(6000);
      } else {
        expect(target).toBe(8000);
      }
    }
  });

  it('keeps a constraint the aircraft may reach when descending steeply', () => {
    // 3000 ft/min at 300 kt: 600 ft/NM, so up to 900 ft/NM: 1500 ft to lose over 2 NM is possible
    expect(findDescentConstraintTarget(108, 9500, -3000, GS, CONSTRAINTS, FCU_ALTITUDE)).toBe(8000);
  });

  it('targets the next constraint when not descending', () => {
    expect(findDescentConstraintTarget(100, 9000, 0, GS, CONSTRAINTS, FCU_ALTITUDE)).toBe(8000);
    expect(findDescentConstraintTarget(100, 9000, null, GS, CONSTRAINTS, FCU_ALTITUDE)).toBe(8000);
  });

  it('skips constraints the aircraft is already at or below', () => {
    expect(findDescentConstraintTarget(105, 7000, VS, GS, CONSTRAINTS, FCU_ALTITUDE)).toBe(6000);
    expect(findDescentConstraintTarget(105, 8010, VS, GS, CONSTRAINTS, FCU_ALTITUDE)).toBe(6000);
  });

  it('skips constraints behind the aircraft', () => {
    expect(findDescentConstraintTarget(111, 7000, VS, GS, CONSTRAINTS, FCU_ALTITUDE)).toBe(6000);
  });

  it('returns null when the FCU altitude is reached first', () => {
    expect(findDescentConstraintTarget(118, 7000, VS, GS, CONSTRAINTS, 6000)).toBeNull();
    expect(findDescentConstraintTarget(141, 3000, VS, GS, CONSTRAINTS, FCU_ALTITUDE)).toBeNull();
    expect(findDescentConstraintTarget(100, 10000, VS, GS, [], FCU_ALTITUDE)).toBeNull();
  });

  it('reports the constraints released on the way to the target', () => {
    const { altitude, released } = find(108, 9500, VS, GS, CONSTRAINTS, FCU_ALTITUDE);
    expect(altitude).toBe(6000);
    expect(released).toEqual([BARET]);
  });

  it('does not move the target back up to a released constraint when the descent steepens', () => {
    // OKAIN 3600 ft 2.2 NM ahead at 5000 ft: released at 600 ft/NM (1.5 * 400 ft/NM)
    const okain: DescentTargetConstraint = { distanceFromStart: 102.2, minimumAltitude: 3600, wasReleased: false };
    const cijmi: DescentTargetConstraint = { distanceFromStart: 105, minimumAltitude: 2700, wasReleased: false };
    const gs = 150;
    const first = find(100, 5000, -1000, gs, [okain, cijmi], 1000);
    expect(first.altitude).toBe(2700);
    expect(first.released).toEqual([okain]);

    // DES then pitches down at 1500 ft/min (600 ft/NM): could reach 3600 at 900 ft/NM, but not at 600 ft/NM
    const released = { ...okain, wasReleased: true };
    expect(find(100, 5000, -1500, gs, [released, cijmi], 1000).altitude).toBe(2700);
    // A never released constraint would be the target at that gradient
    expect(find(100, 5000, -1500, gs, [okain, cijmi], 1000).altitude).toBe(3600);
  });

  it('targets a released constraint again if the present descent would reach it before the waypoint', () => {
    const okain: DescentTargetConstraint = { distanceFromStart: 102.2, minimumAltitude: 3600, wasReleased: true };
    const cijmi: DescentTargetConstraint = { distanceFromStart: 105, minimumAltitude: 2700, wasReleased: false };
    // 2000 ft/min at 150 kt: 800 ft/NM, 1760 ft over 2.2 NM
    expect(find(100, 5000, -2000, 150, [okain, cijmi], 1000).altitude).toBe(3600);
    // Not descending: stays released
    expect(find(100, 5000, 0, 150, [okain, cijmi], 1000).altitude).toBe(2700);
  });
});
