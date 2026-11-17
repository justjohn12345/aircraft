# Airports affected by the T/D loss with a PERF APPR wind

Issue [#11025](https://github.com/flybywiresim/aircraft/issues/11025), fix [#11026](https://github.com/flybywiresim/aircraft/pull/11026).

## The bug

Entering the destination wind on PERF APPR stores it as a descent wind entry at the destination airport elevation.
Below that entry, the wind is scaled from 0 at the runway threshold to 1 at the entry. When the airport elevation and
the threshold elevation are exactly equal, the scaling is 0/0 = NaN. If the descent profile goes below the entry, the
whole profile becomes NaN, the T/D disappears from the ND and MCDU, and managed DES dives at idle to each constraint.

So two conditions must both hold:

1. **Flat runway end:** with the MSFS navdata the airport elevation is the mean of the runway elevations, and the
   threshold elevation is the elevation of that runway end. They are exactly equal when the runways are flat in the
   navdata.
2. **MAP below the airport elevation:** the descent profile ends at the approach's missed approach point (MAP)
   altitude. Only when that is below the airport elevation is the wind computed below the entry. With only a runway
   (no approach) the profile ends at the threshold + 50 ft, which is never affected.

## Results

Scanned 2026-11-17 in MSFS 2024, 215 major airline airports (busiest by passengers plus regional hubs), with the
default MSFS navdata. Navigraph navdata (AIRAC 2611 rev. 3) gave the same runway elevations for every runway end. Only
runways of at least 1800 m are counted.

**2 of 213 airports are affected, 12 of 3821 approaches (0.3 %):**

| Airport | Airport elevation (ft) | Affected approaches (MAP altitude, ft) |
|---|---|---|
| KLAS Las Vegas | 2114.1 | ILS 26L Y/Z, LOC 26L Y/Z, RNAV 26L (2108); ILS 26R Y/Z, LOC 26R Y/Z, RNAV 26R (2104) |
| EDDS Stuttgart | 1260.6 | ILS 25 (1239), RNAV 25 (1231) |

Both confirmed in flight on the stock Development build: the T/D disappeared after entering the PERF APPR wind
with the ILS 26L at KLAS and the ILS 25 at EDDS. All other approaches at these two airports have their MAP above the
airport elevation and are not affected.

Flat runway ends whose approaches all have the MAP above the airport elevation (not affected), checked in flight at
KSFO with the stock build:

| Airport | Airport elevation (ft) | Flat runway ends | Lowest MAP (ft) |
|---|---|---|---|
| KSFO San Francisco | 10.0 | all | 61 |
| PANC Anchorage | 111.4 | all | 181 |
| MKJS Montego Bay | 3.9 | all | 55 |
| TNCM St Maarten | 11.9 | all | 62 |
| EGSS London Stansted | 324.9 | all | 385 |
| LFMN Nice | 13.1 | all | 59 |
| EDDF Frankfurt | 338.1 | all | 354 |
| LPMA Madeira | 168.5 | all | 196 |
| EIDW Dublin | 220.5 | 34 | 252 |
| WSSS Singapore | 26.0 | 02R 20L | 65 |

The other 201 airports have no flat runway end.

## Method

A script loads each airport the way the A32NX MSFS navdata backend does
(`fbw-common/src/systems/navdata/client/backends/Msfs/Mapping.ts`). It computes the airport elevation (mean of the
runway elevations), each threshold elevation and each approach's MAP altitude (as mapped, rounded to feet) with the same
floating point operations, and checks both conditions. It ran in the A32NX MCDU through the Coherent GT debugger.

Airports scanned: KATL KLAX KORD KDFW KDEN KJFK KSFO KSEA KLAS KMCO KEWR KCLT KPHX KIAH KMIA KBOS KMSP KFLL KDTW KPHL
KLGA KBWI KSLC KSAN KIAD KDCA KMDW KTPA KPDX KDAL KSTL KBNA KAUS KOAK KMSY KSMF KSJC KRDU KMCI KCLE KSAT KPIT KIND KCMH
KCVG KBDL KJAX KABQ PHNL PHOG PANC PGUM CYYZ CYVR CYUL CYYC CYEG CYOW CYWG CYHZ MMMX MMUN MMGL MMMY MPTO MROC MKJS TJSJ
MDPC TNCM SBGR SBGL SBSP SBBR SBKP SBCF SBPA SCEL SAEZ SABE SKBO SPJC SEQM SVMI EGLL EGKK EGSS EGCC EGPH EGPF EGBB EIDW
LFPG LFPO LFMN LFLL EHAM EBBR EDDF EDDM EDDB EDDL EDDH EDDK EDDS LSZH LSGG LOWW LOWI EKCH ENGM ESSA EFHK LEMD LEBL LEPA
LEMG LPPT LPPR LPMA LIRF LIMC LIPZ LGAV LTFM LTFJ LTAI EPWA LKPR LHBP LROP LDZA LCLK LMML EVRA EYVI EETN BIKF UUEE UUDD
ULLI OMDB OMAA OTHH OERK OEJN OBBI OKKK OOMS LLBG OJAI FAOR FACT FALE HECA GMMN DNMM HKJK HAAB DTTA RJTT RJAA RJBB RJGG
RJCC RJFF RKSI RKSS RCTP VHHH VMMC ZBAA ZBAD ZSPD ZSSS ZGGG ZGSZ ZUUU ZUTF ZPPP ZLXY ZHHH ZSHC WSSS WMKK WIII WADD VTBS
VTBD VTSP VVNB VVTS RPLL VIDP VABB VOBL VOMM VOHS VECC OPKC OPLA VCBI VRMM VNKT YSSY YMML YBBN YPPH YSCB NZAA NZCH NZWN
NFFN. EGLC and KSNA are not counted (no runway of 1800 m or more).
