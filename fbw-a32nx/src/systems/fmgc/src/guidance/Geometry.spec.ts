// Copyright (c) 2026 FlyByWire Simulations
//
// SPDX-License-Identifier: GPL-3.0

import { describe, expect, it } from 'vitest';
import { Fix, TurnDirection } from '@flybywiresim/fbw-sdk';
import { placeBearingDistance } from 'msfs-geo';
import { SegmentType } from '@fmgc/flightplanning/FlightPlanSegment';
import { FlightPlanLegDefinition } from '@fmgc/flightplanning/legs/FlightPlanLegDefinition';
import { Geometry } from '@fmgc/guidance/Geometry';
import { CALeg } from '@fmgc/guidance/lnav/legs/CA';
import { CFLeg } from '@fmgc/guidance/lnav/legs/CF';
import { DFLeg } from '@fmgc/guidance/lnav/legs/DF';
import { IFLeg } from '@fmgc/guidance/lnav/legs/IF';
import { LegMetadata } from '@fmgc/guidance/lnav/legs/index';
import { Leg } from '@fmgc/guidance/lnav/legs/Leg';
import { Guidable } from '@fmgc/guidance/Guidable';
import { Transition } from '@fmgc/guidance/lnav/Transition';
import { TransitionPicker } from '@fmgc/guidance/lnav/TransitionPicker';
import { CourseCaptureTransition } from '@fmgc/guidance/lnav/transitions/CourseCaptureTransition';

const TAS = 230;
const GS = 230;

const metadata: LegMetadata = {
  flightPlanLegDefinition: {} as FlightPlanLegDefinition,
  turnDirection: TurnDirection.Either,
};

function fix(ident: string, lat: number, long: number): Fix {
  return { ident, location: { lat, long } } as unknown as Fix;
}

const CA_LEG_INDEX = 2;

/**
 * Builds the geometry of a SID: runway, `firstLeg`, a CA leg and a DF leg, with the CA leg active.
 * @returns the geometry and the CA leg
 */
function buildGeometryWithActiveCaLeg(
  runway: Fix,
  firstLeg: CFLeg | DFLeg,
  caCourse: DegreesTrue,
  caAltitude: Feet,
  nextFix: Fix,
): [Geometry, CALeg] {
  const runwayLeg = new IFLeg(runway, metadata, SegmentType.Departure);
  const caLeg = new CALeg(caCourse, caAltitude, metadata, SegmentType.Departure);
  const dfLeg = new DFLeg(nextFix, metadata, SegmentType.Departure);

  const toCa = TransitionPicker.forLegs(firstLeg, caLeg)!;
  const fromCa = TransitionPicker.forLegs(caLeg, dfLeg)!;

  const legs = new Map<number, Leg>([
    [0, runwayLeg],
    [1, firstLeg],
    [CA_LEG_INDEX, caLeg],
    [3, dfLeg],
  ]);
  const transitions = new Map<number, Transition>([
    [1, toCa],
    [CA_LEG_INDEX, fromCa],
  ]);

  const none = null as unknown as Guidable;
  runwayLeg.setNeighboringGuidables(none, firstLeg);
  firstLeg.setNeighboringGuidables(runwayLeg, toCa);
  toCa.setNeighboringGuidables(firstLeg, caLeg);
  caLeg.setNeighboringGuidables(toCa, fromCa);
  fromCa.setNeighboringGuidables(caLeg, dfLeg);
  dfLeg.setNeighboringGuidables(fromCa, none);

  const guidables: Guidable[] = [runwayLeg, firstLeg, toCa, caLeg, fromCa, dfLeg];

  // First as computed before takeoff, then with the CA leg active
  for (const isCaActive of [false, true]) {
    for (const guidable of guidables) {
      guidable.recomputeWithParameters(guidable === caLeg && isCaActive, TAS, GS, runway.location, caCourse);
    }
  }

  return [new Geometry(transitions, legs, false), caLeg];
}

describe('Geometry.shouldSequenceLeg', () => {
  describe('CA leg after a course capture transition', () => {
    // OTHH ALSE2E RWY 34R: CF ATBIB 336M, CA 336M to 500 ft, DF REVIN. A small difference between the two true courses
    // makes the course capture transition into the CA leg non-null, which used to prevent the CA leg from sequencing.
    const othh34r = fix('RW34R', 25.255458, 51.626781);
    const atbib = fix('ATBIB', 25.32715, 51.595194);
    const revin = fix('REVIN', 25.469192, 51.631856);
    const othhCaCourse = 338.6;

    const buildOthh = () =>
      buildGeometryWithActiveCaLeg(
        othh34r,
        new CFLeg(atbib, 338.2, 2, metadata, SegmentType.Departure),
        othhCaCourse,
        500,
        revin,
      );

    it('has a non-null course capture transition into the CA leg', () => {
      const [geometry] = buildOthh();

      const transition = geometry.transitions.get(CA_LEG_INDEX - 1);
      expect(transition).toBeInstanceOf(CourseCaptureTransition);
      expect(transition!.isNull).toBe(false);
    });

    it('does not sequence the CA leg before its estimated termination', () => {
      const [geometry] = buildOthh();

      const ppos = placeBearingDistance(atbib.location, othhCaCourse, 0.1);

      expect(geometry.shouldSequenceLeg(CA_LEG_INDEX, { ...ppos, alt: 1000 })).toBe(false);
    });

    it('sequences the CA leg once its estimated termination has been passed', () => {
      const [geometry] = buildOthh();

      // About where the aircraft was in the sim, still on the CA leg (2.4 NM past its termination)
      const ppos = placeBearingDistance(atbib.location, othhCaCourse, 3);

      expect(geometry.transitions.get(CA_LEG_INDEX - 1)!.isAbeam(ppos)).toBe(true);
      expect(geometry.shouldSequenceLeg(CA_LEG_INDEX, { ...ppos, alt: 4000 })).toBe(true);
    });

    it('sequences a CA leg after a DF leg once its estimated termination has been passed', () => {
      // NZCH ATSA1Q RWY 20: DF DER20, CA 196M to 520 ft, CF ATSAT. The DF course depends on where it starts.
      const nzch20 = fix('RW20', -43.474989, 172.548286);
      const der20 = fix('DER20', -43.497631, 172.522114);
      const atsat = fix('ATSAT', -43.583794, 172.352322);
      const caCourse = 220.1;

      const [geometry] = buildGeometryWithActiveCaLeg(
        nzch20,
        new DFLeg(der20, metadata, SegmentType.Departure),
        caCourse,
        520,
        atsat,
      );

      expect(geometry.transitions.get(CA_LEG_INDEX - 1)!.isNull).toBe(false);

      const ppos = placeBearingDistance(der20.location, caCourse, 3);

      expect(geometry.shouldSequenceLeg(CA_LEG_INDEX, { ...ppos, alt: 4000 })).toBe(true);
    });
  });
});
