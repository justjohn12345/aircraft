# Airports affected by the T/D loss with a PERF APPR wind

Issue [#11025](https://github.com/flybywiresim/aircraft/issues/11025), fix [#11026](https://github.com/flybywiresim/aircraft/pull/11026).

## The bug

Entering the destination wind on PERF APPR stores it as a descent wind entry at the destination airport elevation.
Below that entry, the wind is scaled from 0 at the runway threshold to 1 at the entry. When the airport elevation and
the threshold elevation are exactly equal, the scaling is 0/0 = NaN, the whole descent profile becomes NaN, the T/D
disappears from the ND and MCDU, and managed DES dives at idle to each constraint.

With the MSFS navdata the airport elevation is the mean of the runway elevations, and the threshold elevation is the
elevation of that runway end. They are exactly equal when the runways are flat in the navdata, which is common.

## Results

Scanned 2026-11-17 in MSFS 2024 with Navigraph navdata (AIRAC 2611 rev. 3), 215 major airline airports (busiest by
passengers plus regional hubs). Only runways of at least 1800 m are counted.

**12 of 212 airports are affected, 47 of 1010 runway ends (4.7 %).**

| Airport | Airport elevation (ft) | Affected runway ends | Unaffected runway ends (threshold elevation, ft) |
|---|---|---|---|
| KSFO San Francisco | 10.0 | all: 01L 19R 01R 19L 10L 28R 10R 28L | |
| KLAS Las Vegas | 2114.1 | all: 08L 26R 08R 26L 01R 19L 01L 19R | |
| PANC Anchorage | 111.4 | all: 07L 25R 07R 25L 15 33 | |
| MKJS Montego Bay | 3.9 | all: 07 25 | |
| TNCM St Maarten | 11.9 | all: 10 28 | |
| EGSS London Stansted | 324.9 | all: 04 22 | |
| LFMN Nice | 13.1 | all: 04L 22R 04R 22L | |
| EDDF Frankfurt | 338.1 | all: 07L 25R 07C 25C 07R 25L 18 36 | |
| EDDS Stuttgart | 1260.6 | all: 07 25 | |
| LPMA Madeira | 168.5 | all: 05 23 | |
| EIDW Dublin | 220.5 | 34 | 10L (235), 28R (208), 10R (239), 28L (221), 16 (221) |
| WSSS Singapore | 26.0 | 02R 20L | 02L (25), 20R (26), 02C (27), 20C (25) |

At EIDW, 28L and 16 also show 221 ft, but they differ from the airport elevation by a fraction of a foot, which is
enough: only an exact match gives the NaN.

## Method

A script loads each airport the way the A32NX MSFS navdata backend does
(`fbw-common/src/systems/navdata/client/backends/Msfs/Mapping.ts`), computes the airport elevation (mean of the runway
elevations) and each threshold elevation with the same floating point operations, and counts the runway ends where they
are exactly equal. It ran in the A32NX MCDU through the Coherent GT debugger.

Not affected (no runway end with an exact match): KATL KLAX KORD KDFW KDEN KJFK KSEA KMCO KEWR KCLT KPHX KIAH KMIA
KBOS KMSP KFLL KDTW KPHL KLGA KBWI KSLC KSAN KIAD KDCA KMDW KTPA KPDX KDAL KSTL KBNA KAUS KOAK KMSY KSMF KSJC KRDU
KMCI KCLE KSAT KPIT KIND KCMH KCVG KBDL KJAX KABQ PHNL PHOG PGUM CYYZ CYVR CYUL CYYC CYEG CYOW CYWG CYHZ MMMX MMUN
MMGL MMMY MPTO MROC TJSJ MDPC SBGR SBGL SBSP SBBR SBKP SBCF SBPA SCEL SAEZ SABE SKBO SPJC SEQM SVMI EGLL EGKK EGCC
EGPH EGPF EGBB LFPG LFPO LFLL EHAM EBBR EDDM EDDB EDDL EDDH EDDK LSZH LSGG LOWW LOWI EKCH ENGM ESSA EFHK LEMD LEBL
LEPA LEMG LPPT LPPR LIRF LIMC LIPZ LGAV LTFM LTFJ LTAI EPWA LKPR LHBP LROP LDZA LCLK LMML EVRA EYVI EETN BIKF UUEE
UUDD ULLI OMDB OMAA OTHH OERK OEJN OBBI OOMS LLBG OJAI FAOR FACT FALE HECA GMMN DNMM HKJK HAAB DTTA RJTT RJAA RJBB
RJGG RJCC RJFF RKSI RKSS RCTP VHHH VMMC ZBAA ZBAD ZSPD ZSSS ZGGG ZGSZ ZUUU ZUTF ZPPP ZLXY ZHHH ZSHC WMKK WIII WADD
VTBS VTBD VTSP VVNB VVTS RPLL VIDP VABB VOBL VOMM VOHS VECC OPKC OPLA VCBI VRMM VNKT YSSY YMML YBBN YPPH YSCB NZAA
NZCH NZWN NFFN.

Not counted: EGLC and KSNA (no runway of 1800 m or more); OKKK not scanned (wrong ident in the list).
