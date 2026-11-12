// Copyright (c) 2026 FlyByWire Simulations
//
// SPDX-License-Identifier: GPL-3.0

import { describe, expect, it } from 'vitest';
import {
  interpolatePathAltitude,
  smoothApproachPath,
  SmoothingConstraint,
  SmoothingPoint,
} from '@fmgc/guidance/vnav/descent/ApproachPathSmoother';

const speed = () => 200;

// Shaped like the NZQN RNP Y profile: level decelerations at each constraint and a coded level segment QN780-TOGOX.
const ATKIL = 10;
const QN780 = 14;
const TOGOX = 18;
const QN770 = 22;
const ATVUP = 25;
const QN760 = 30;
// The coded final descent starts after QN760 in this example
const FDP = 31;

const constraints: SmoothingConstraint[] = [
  { distanceFromStart: ATKIL, minimumAltitude: 10000 },
  { distanceFromStart: QN780, minimumAltitude: 8600 },
  { distanceFromStart: TOGOX, minimumAltitude: 8600 },
  { distanceFromStart: QN770, minimumAltitude: 7500 },
  { distanceFromStart: ATVUP, minimumAltitude: 6700 },
  { distanceFromStart: QN760, minimumAltitude: 5100 },
];

const original: SmoothingPoint[] = [
  { distanceFromStart: 0, altitude: 13000 },
  { distanceFromStart: ATKIL, altitude: 10000 },
  { distanceFromStart: 13.4, altitude: 8600 },
  { distanceFromStart: QN780, altitude: 8600 },
  { distanceFromStart: TOGOX, altitude: 8600 },
  { distanceFromStart: 21.7, altitude: 7500 },
  { distanceFromStart: 22.3, altitude: 7500 },
  { distanceFromStart: 24.6, altitude: 6700 },
  { distanceFromStart: 25.9, altitude: 6700 },
  { distanceFromStart: 29.2, altitude: 5100 },
  { distanceFromStart: 30.8, altitude: 5100 },
  { distanceFromStart: 36, altitude: 3300 },
];

function gradients(path: readonly SmoothingPoint[], from: number, to: number): number[] {
  const result: number[] = [];
  for (let i = 1; i < path.length; i++) {
    if (path[i - 1].distanceFromStart >= from && path[i].distanceFromStart <= to) {
      result.push(
        (path[i - 1].altitude - path[i].altitude) / (path[i].distanceFromStart - path[i - 1].distanceFromStart),
      );
    }
  }
  return result;
}

describe('smoothApproachPath', () => {
  const smoothed = smoothApproachPath(original, constraints, ATKIL, FDP, speed);

  it('crosses each constraint inside a removed level segment at the constraint altitude', () => {
    expect(interpolatePathAltitude(smoothed, QN770)).toBeCloseTo(7500, -1);
    expect(interpolatePathAltitude(smoothed, ATVUP)).toBeGreaterThanOrEqual(6700);
    expect(interpolatePathAltitude(smoothed, ATVUP)).toBeLessThan(6760);
    expect(interpolatePathAltitude(smoothed, QN760)).toBeGreaterThanOrEqual(5100);
  });

  it('removes the level segments around single constraints', () => {
    // Originally level at 7500 from 21.7 to 22.3 and at 6700 from 24.6 to 25.9
    expect(interpolatePathAltitude(smoothed, 21.75)).toBeGreaterThan(7520);
    expect(interpolatePathAltitude(smoothed, 22.25)).toBeLessThan(7480);
    expect(interpolatePathAltitude(smoothed, 25.85)).toBeLessThan(6650);
  });

  it('keeps a coded level segment between two constraints, but not the early level-off before it', () => {
    expect(interpolatePathAltitude(smoothed, 16)).toBeCloseTo(8600, 0);
    expect(interpolatePathAltitude(smoothed, 13.4)).toBeGreaterThan(8620);
  });

  it('never goes below an at-or-above constraint', () => {
    for (const constraint of constraints) {
      expect(interpolatePathAltitude(smoothed, constraint.distanceFromStart)).toBeGreaterThanOrEqual(
        constraint.minimumAltitude - 1,
      );
    }
  });

  it('does not change the path before the approach', () => {
    expect(interpolatePathAltitude(smoothed, 5)).toBeCloseTo(interpolatePathAltitude(original, 5), 6);
  });

  it('changes the path angle in small steps', () => {
    const steps = gradients(smoothed, ATKIL, 36);
    let largestStep = 0;
    for (let i = 1; i < steps.length; i++) {
      largestStep = Math.max(largestStep, Math.abs(steps[i] - steps[i - 1]));
    }

    // The original corners change by up to ~480 ft/NM at once
    expect(largestStep).toBeLessThan(200);
  });

  it('spreads a level-off over a longer distance than a steepening of the same size', () => {
    const levelling: SmoothingPoint[] = [
      { distanceFromStart: 0, altitude: 6000 },
      { distanceFromStart: 10, altitude: 3000 },
      { distanceFromStart: 20, altitude: 3000 },
    ];
    const steepening: SmoothingPoint[] = [
      { distanceFromStart: 0, altitude: 6000 },
      { distanceFromStart: 10, altitude: 6000 },
      { distanceFromStart: 20, altitude: 3000 },
    ];

    const span = (path: SmoothingPoint[]) => {
      const changed = path.filter((p) => p.distanceFromStart > 0 && p.distanceFromStart < 20);
      return changed[changed.length - 1].distanceFromStart - changed[0].distanceFromStart;
    };

    const levellingSpan = span(smoothApproachPath(levelling, [], 0, null, speed));
    const steepeningSpan = span(smoothApproachPath(steepening, [], 0, null, speed));

    expect(levellingSpan).toBeGreaterThan(steepeningSpan * 1.5);
  });

  it('keeps level segments that are not in the middle of a descent', () => {
    const path: SmoothingPoint[] = [
      { distanceFromStart: 0, altitude: 5000 },
      { distanceFromStart: 10, altitude: 5000 },
      { distanceFromStart: 20, altitude: 2000 },
    ];

    expect(interpolatePathAltitude(smoothApproachPath(path, [], 0, null, speed), 5)).toBeCloseTo(5000, 6);
  });

  describe('pulling the path down to the approach constraints', () => {
    // As planned with decelerations: ATKIL crossed at 10750, QN780 at 9600, 8600 only reached just before TOGOX
    const high: SmoothingPoint[] = [
      { distanceFromStart: 0, altitude: 13000 },
      { distanceFromStart: ATKIL - 5, altitude: 11300 },
      { distanceFromStart: ATKIL, altitude: 10750 },
      { distanceFromStart: QN780, altitude: 9600 },
      { distanceFromStart: 17.5, altitude: 8600 },
      { distanceFromStart: TOGOX, altitude: 8600 },
      { distanceFromStart: QN770, altitude: 7500 },
      { distanceFromStart: ATVUP, altitude: 6700 },
      { distanceFromStart: QN760, altitude: 5100 },
      { distanceFromStart: 36, altitude: 3300 },
    ];
    const pulled = smoothApproachPath(high, constraints, ATKIL, FDP, speed);

    it('crosses the initial approach fix and the following constraints at their altitudes', () => {
      expect(interpolatePathAltitude(pulled, ATKIL)).toBeCloseTo(10000, -1);
      expect(interpolatePathAltitude(pulled, QN780)).toBeGreaterThanOrEqual(8600);
      expect(interpolatePathAltitude(pulled, QN780)).toBeLessThan(8660);
      expect(interpolatePathAltitude(pulled, 16)).toBeCloseTo(8600, 0);
      expect(interpolatePathAltitude(pulled, TOGOX)).toBeCloseTo(8600, 0);
    });

    it('never goes below a constraint', () => {
      for (const constraint of constraints) {
        expect(interpolatePathAltitude(pulled, constraint.distanceFromStart)).toBeGreaterThanOrEqual(
          constraint.minimumAltitude - 1,
        );
      }
    });

    it('joins the earlier path without a steep segment', () => {
      expect(interpolatePathAltitude(pulled, 0)).toBeCloseTo(13000, 6);
      for (const gradient of gradients(pulled, ATKIL - 5, ATKIL)) {
        expect(gradient).toBeLessThanOrEqual(300 + 1);
      }
    });

    it('does not pull down to constraints on the coded final descent', () => {
      // The FDP is before QN760, which the final path crosses above its altitude
      const final = smoothApproachPath(high, constraints, ATKIL, ATVUP + 0.5, speed);
      expect(interpolatePathAltitude(final, QN760)).toBeCloseTo(interpolatePathAltitude(high, QN760), 0);
    });

    it('stays above a constraint that would need a too steep segment', () => {
      const steep: SmoothingPoint[] = [
        { distanceFromStart: 0, altitude: 12000 },
        { distanceFromStart: 10, altitude: 11000 },
        { distanceFromStart: 11, altitude: 10000 },
        { distanceFromStart: 20, altitude: 7000 },
      ];
      const result = smoothApproachPath(
        steep,
        [
          { distanceFromStart: 10, minimumAltitude: 9000 },
          { distanceFromStart: 11, minimumAltitude: 8000 },
        ],
        10,
        12,
        speed,
      );
      for (const gradient of gradients(result, 10, 12)) {
        expect(gradient).toBeLessThanOrEqual(400 + 1);
      }
      expect(interpolatePathAltitude(result, 10)).toBeGreaterThanOrEqual(9000);
    });

    it('does nothing without an FDP', () => {
      const unchanged = smoothApproachPath(high, constraints, ATKIL, null, speed);
      expect(interpolatePathAltitude(unchanged, ATKIL)).toBeCloseTo(10750, 0);
    });
  });
});
