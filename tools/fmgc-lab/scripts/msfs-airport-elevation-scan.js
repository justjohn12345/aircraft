// Copyright (c) 2026 FlyByWire Simulations
//
// SPDX-License-Identifier: GPL-3.0

// Findings C17 (issue #11025): how many airports in the installed MSFS navdata lose the T/D with a PERF APPR wind.
//
// The MSFS navdata backend (fbw-common/src/systems/navdata/client/backends/Msfs/Mapping.ts) makes the airport
// elevation the mean of the runway elevations and the threshold elevation the runway end elevation. The PERF APPR wind
// is entered at the airport elevation and scaled down to the destination runway threshold; when the two are exactly
// equal the scaling is 0/0 and the descent profile goes NaN. This script loads airports the same way the backend does
// and counts the runway ends where they are exactly equal (computed in the same float operations).
//
// Usage: developer mode, Coherent GT debugger (port 19999), open any FBW instrument (e.g. the A32NX MCDU), paste this
// into the console. Best on the ground or paused: loading thousands of airports stalls the instrument for a while.
// Edit CONFIG first. Results: console summary, console.table of the affected airports, and window.fbwElevationScan
// (.csv holds one line per checked runway end).

(async () => {
  // Major airline airports (busiest by passengers plus regional hubs), for a quick first scan.
  const MAJOR_AIRPORTS = [
    // USA, Canada
    'KATL',
    'KLAX',
    'KORD',
    'KDFW',
    'KDEN',
    'KJFK',
    'KSFO',
    'KSEA',
    'KLAS',
    'KMCO',
    'KEWR',
    'KCLT',
    'KPHX',
    'KIAH',
    'KMIA',
    'KBOS',
    'KMSP',
    'KFLL',
    'KDTW',
    'KPHL',
    'KLGA',
    'KBWI',
    'KSLC',
    'KSAN',
    'KIAD',
    'KDCA',
    'KMDW',
    'KTPA',
    'KPDX',
    'KDAL',
    'KSTL',
    'KBNA',
    'KAUS',
    'KOAK',
    'KMSY',
    'KSMF',
    'KSJC',
    'KRDU',
    'KMCI',
    'KSNA',
    'KCLE',
    'KSAT',
    'KPIT',
    'KIND',
    'KCMH',
    'KCVG',
    'KBDL',
    'KJAX',
    'KABQ',
    'PHNL',
    'PHOG',
    'PANC',
    'PGUM',
    'CYYZ',
    'CYVR',
    'CYUL',
    'CYYC',
    'CYEG',
    'CYOW',
    'CYWG',
    'CYHZ',
    // Mexico, Central America, Caribbean, South America
    'MMMX',
    'MMUN',
    'MMGL',
    'MMMY',
    'MPTO',
    'MROC',
    'MKJS',
    'TJSJ',
    'MDPC',
    'TNCM',
    'SBGR',
    'SBGL',
    'SBSP',
    'SBBR',
    'SBKP',
    'SBCF',
    'SBPA',
    'SCEL',
    'SAEZ',
    'SABE',
    'SKBO',
    'SPJC',
    'SEQM',
    'SVMI',
    // Europe
    'EGLL',
    'EGKK',
    'EGSS',
    'EGCC',
    'EGLC',
    'EGPH',
    'EGPF',
    'EGBB',
    'EIDW',
    'LFPG',
    'LFPO',
    'LFMN',
    'LFLL',
    'EHAM',
    'EBBR',
    'EDDF',
    'EDDM',
    'EDDB',
    'EDDL',
    'EDDH',
    'EDDK',
    'EDDS',
    'LSZH',
    'LSGG',
    'LOWW',
    'LOWI',
    'EKCH',
    'ENGM',
    'ESSA',
    'EFHK',
    'LEMD',
    'LEBL',
    'LEPA',
    'LEMG',
    'LPPT',
    'LPPR',
    'LPMA',
    'LIRF',
    'LIMC',
    'LIPZ',
    'LGAV',
    'LTFM',
    'LTFJ',
    'LTAI',
    'EPWA',
    'LKPR',
    'LHBP',
    'LROP',
    'LDZA',
    'LCLK',
    'LMML',
    'EVRA',
    'EYVI',
    'EETN',
    'BIKF',
    'UUEE',
    'UUDD',
    'ULLI',
    // Middle East, Africa
    'OMDB',
    'OMAA',
    'OTHH',
    'OERK',
    'OEJN',
    'OBBI',
    'OKKK',
    'OOMS',
    'LLBG',
    'OJAI',
    'FAOR',
    'FACT',
    'FALE',
    'HECA',
    'GMMN',
    'DNMM',
    'HKJK',
    'HAAB',
    'DTTA',
    // Asia, Oceania
    'RJTT',
    'RJAA',
    'RJBB',
    'RJGG',
    'RJCC',
    'RJFF',
    'RKSI',
    'RKSS',
    'RCTP',
    'VHHH',
    'VMMC',
    'ZBAA',
    'ZBAD',
    'ZSPD',
    'ZSSS',
    'ZGGG',
    'ZGSZ',
    'ZUUU',
    'ZUTF',
    'ZPPP',
    'ZLXY',
    'ZHHH',
    'ZSHC',
    'WSSS',
    'WMKK',
    'WIII',
    'WADD',
    'VTBS',
    'VTBD',
    'VTSP',
    'VVNB',
    'VVTS',
    'RPLL',
    'VIDP',
    'VABB',
    'VOBL',
    'VOMM',
    'VOHS',
    'VECC',
    'OPKC',
    'OPLA',
    'VCBI',
    'VRMM',
    'VNKT',
    'YSSY',
    'YMML',
    'YBBN',
    'YPPH',
    'YSCB',
    'NZAA',
    'NZCH',
    'NZWN',
    'NFFN',
  ];

  const CONFIG = {
    // Airports to check (ICAO idents): MAJOR_AIRPORTS, your own list, or [] to search every airport by prefix.
    idents: MAJOR_AIRPORTS,
    // Ident prefixes to search when idents is empty. One letter covers a region (K = USA, E/L = Europe, ...).
    prefixes: 'ABCDEFGHIJKLMNOPQRSTUVWXYZ'.split(''),
    maxPerPrefix: 20000,
    // Only runway ends of runways at least this long count (airliner airports). 0 for all.
    minRunwayLengthM: 1800,
    batchSize: 100,
    timeoutMs: 10000,
  };

  const DESIGNATOR = ['', 'L', 'R', 'C', 'W', 'A', 'B'];
  const ident = (icao) => icao.substring(7).trim();
  const waiters = new Map();
  const sub = Coherent.on('SendAirport', (f) => {
    const w = f && f.icao ? waiters.get(ident(f.icao)) : undefined;
    if (w) {
      waiters.delete(ident(f.icao));
      w(f);
    }
  });

  const loadBatch = async (icaos) => {
    const pending = icaos.map(
      (icao) =>
        new Promise((resolve) => {
          waiters.set(ident(icao), resolve);
          setTimeout(() => {
            if (waiters.delete(ident(icao))) {
              resolve(null);
            }
          }, CONFIG.timeoutMs);
        }),
    );
    const ok = await Coherent.call('LOAD_AIRPORTS', icaos);
    icaos.forEach((icao, i) => {
      const w = waiters.get(ident(icao));
      if (!(ok && ok[i]) && w) {
        waiters.delete(ident(icao));
        w(null);
      }
    });
    return Promise.all(pending);
  };

  // Airport ICAO strings: 'A' + 6 blanks (region, airport) + ident.
  let icaos = [];
  if (CONFIG.idents.length > 0) {
    icaos = CONFIG.idents.map((id) => `A      ${id.toUpperCase()}`);
  } else {
    for (const prefix of CONFIG.prefixes) {
      const found = (await Coherent.call('SEARCH_BY_IDENT', prefix, 1, CONFIG.maxPerPrefix)) || [];
      const airports = found.filter((icao) => icao.charAt(0) === 'A');
      console.log(
        `prefix ${prefix}: ${airports.length} airports${airports.length >= CONFIG.maxPerPrefix ? ' (CAPPED)' : ''}`,
      );
      icaos.push(...airports);
    }
  }
  icaos = [...new Set(icaos)];
  console.log(`loading ${icaos.length} airports`);

  const lines = ['airport,runway_end,airport_alt_ft,threshold_alt_ft,runway_length_m,affected'];
  const affectedAirports = [];
  let airportsChecked = 0;
  let endsChecked = 0;
  let endsAffected = 0;
  const missing = [];

  for (let i = 0; i < icaos.length; i += CONFIG.batchSize) {
    const facilities = await loadBatch(icaos.slice(i, i + CONFIG.batchSize));
    for (let j = 0; j < facilities.length; j++) {
      const ap = facilities[j];
      if (!ap || !ap.runways || ap.runways.length === 0) {
        missing.push(ident(icaos[i + j]));
        continue;
      }
      // Same as Mapping.mapAirport: mean of runway.elevation / 0.3048.
      const elevations = ap.runways.map((r) => r.elevation / 0.3048);
      const airportAlt = elevations.reduce((a, b) => a + b, 0) / elevations.length;

      const ends = [];
      for (const r of ap.runways) {
        if (r.length < CONFIG.minRunwayLengthM) {
          continue;
        }
        r.designation.split('-').forEach((num, index) => {
          const primary = index === 0;
          const char = DESIGNATOR[primary ? r.designatorCharPrimary : r.designatorCharSecondary];
          if (char === 'W' || char === undefined) {
            return;
          }
          // Same as Mapping: thresholdLocation.alt.
          const thresholdAlt = (primary ? r.primaryElevation : r.secondaryElevation) / 0.3048;
          const affected = thresholdAlt === airportAlt;
          ends.push({ name: `${num.padStart(2, '0')}${char}`, affected });
          lines.push(
            `${ident(ap.icao)},${num.padStart(2, '0')}${char},${airportAlt},${thresholdAlt},${r.length},${affected ? 1 : 0}`,
          );
        });
      }
      if (ends.length === 0) {
        continue;
      }
      airportsChecked++;
      endsChecked += ends.length;
      const hit = ends.filter((e) => e.affected);
      endsAffected += hit.length;
      if (hit.length > 0) {
        affectedAirports.push({
          airport: ident(ap.icao),
          elevationFt: Math.round(airportAlt),
          runways: ap.runways.length,
          affectedEnds: hit.map((e) => e.name).join(' '),
          allEnds: hit.length === ends.length,
        });
      }
    }
    if ((i / CONFIG.batchSize) % 10 === 0) {
      console.log(`${Math.min(i + CONFIG.batchSize, icaos.length)} / ${icaos.length}`);
    }
  }

  if (sub && sub.clear) {
    sub.clear();
  }

  const pct = (a, b) => (b > 0 ? ((100 * a) / b).toFixed(1) : '0');
  console.log(
    `airports with a runway >= ${CONFIG.minRunwayLengthM} m: ${airportsChecked}, affected: ${affectedAirports.length} ` +
      `(${pct(affectedAirports.length, airportsChecked)} %); runway ends: ${endsChecked}, affected: ${endsAffected} ` +
      `(${pct(endsAffected, endsChecked)} %); not loaded: ${missing.length}${missing.length > 0 ? ` (${missing.join(' ')})` : ''}`,
  );
  console.table(affectedAirports);
  window.fbwElevationScan = { affectedAirports, csv: lines.join('\n') };
  console.log('full results: window.fbwElevationScan (.csv per runway end)');
})();
