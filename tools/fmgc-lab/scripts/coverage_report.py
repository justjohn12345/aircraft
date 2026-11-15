#!/usr/bin/env python3
# Summarises an `llvm-cov export -format=text` JSON of the generated model: line / region / branch coverage per file,
# and every branch with a side never taken, grouped by subsystem using the line map of docs/structure.md (stock master
# line numbers). Optionally compares with a baseline export (branch sides newly covered).
#
# usage: coverage_report.py export.json [--baseline other.json] [--source-dir build/models/stock] > uncovered.txt

import argparse
import collections
import json
import os
import sys

# (first line, last line, subsystem), docs/structure.md "Corrections" (FmgcComputer.cpp, stock master)
COMPUTER = [
    (1, 200, "block library helpers (MATLABFunction*, filters, ppval)"),
    (201, 450, "step(): locals"),
    (451, 694, "(re)initialisation on the first running frame"),
    (695, 720, "conditioning: ADR validity and vote"),
    (721, 771, "conditioning: IR validity and vote"),
    (772, 864, "conditioning: ADR selection, indicated altitude"),
    (865, 1075, "conditioning: IR selection"),
    (1076, 1100, "conditioning: RA"),
    (1101, 1241, "conditioning: FAC selection, rudder control"),
    (1242, 1307, "conditioning: FAC speeds/weights, on ground, 5 s flight, flap lever"),
    (1308, 1313, "conditioning: engines, FCU"),
    (1314, 1349, "conditioning: ILS"),
    (1350, 1364, "conditioning: TCAS"),
    (1365, 1379, "conditioning: ILS tune inhibit, runway heading memo"),
    (1380, 1559, "engagement: AP/FD/A/THR engagement, opp sync, mode reset"),
    (1560, 1635, "vertical prelude: TCAS RA decoding, TRK-FPA deselection"),
    (1636, 1664, "altitude target (alt_cstr_applicable, alt_sel_or_cstr)"),
    (1665, 1721, "SRS takeoff"),
    (1722, 1793, "lateral: RWY, RWY TRK"),
    (1794, 1912, "lateral/approach: LAND armed, LAND, ROLL OUT, ALIGN"),
    (1913, 1920, "approach: FLARE"),
    (1921, 1990, "approach: FINAL armed, FINAL"),
    (1991, 2053, "SRS go-around, GA TRK"),
    (2054, 2142, "lateral: mode reversion, HDG/TRK"),
    (2143, 2241, "lateral: NAV armed, NAV, LOC armed"),
    (2242, 2404, "lateral: LOC*, LOC"),
    (2405, 2464, "vertical: ALT arm possible, ALT armed"),
    (2465, 2549, "vertical: ALT*"),
    (2550, 2611, "vertical: ALT, ALT CRZ"),
    (2612, 2720, "vertical: V/S reversion, V/S-FPA"),
    (2721, 2898, "vertical: CLB armed, CLB, DES armed"),
    (2899, 3150, "vertical: DES, OP CLB, OP DES, EXP CLB, EXP DES"),
    (3151, 3262, "approach: G/S armed, G/S*, G/S"),
    (3263, 3334, "vertical: TCAS armed, TCAS"),
    (3335, 3404, "engagement stage 2: FD auto disengage, V/S not held, reversion, FD bars"),
    (3405, 3434, "targets: HDG/TRK preset"),
    (3435, 3536, "targets: selected/managed speed, SPD/MACH"),
    (3537, 3659, "targets: speed memos, SRS targets, DES margin"),
    (3660, 3712, "targets: speed target selection"),
    (3713, 3735, "targets: ALT soft"),
    (3736, 3795, "law selection"),
    (3796, 3863, "LAND 2/3 capability"),
    (3864, 4137, "logic / ap_fd_logic output assembly"),
    (4138, 4179, "outer loops call"),
    (4180, 4424, "autothrust: modes, alpha floor, FMA"),
    (4425, 4641, "autothrust: N1 command"),
    (4642, 4714, "output packing: value-word SSMs"),
    (4715, 4925, "output packing: discrete words"),
    (4926, 5133, "outputs and end-of-step delays"),
    (5134, 99999, "initialize() / terminate()"),
]
OUTER = [
    (1, 2017, "outer-loop block library helpers (filters, limiters, charts)"),
    (2018, 2150, "init() / reset()"),
    (2151, 2231, "step(): locals"),
    (2232, 2275, "lateral: bank limit"),
    (2276, 2289, "lateral: NAV feedforward"),
    (2290, 2330, "lateral: HDG"),
    (2331, 2370, "lateral: LOC_CPT, Phi_loc_c"),
    (2371, 2411, "lateral: LOC_TRACK filters, runway alignment"),
    (2412, 2442, "lateral: TRACK"),
    (2443, 2531, "lateral: roll law switch (HPATH, LOC_TRACK)"),
    (2532, 2540, "lateral: FD roll"),
    (2541, 2626, "lateral: ROLL_OUT"),
    (2627, 2716, "lateral: sideslip, nosewheel"),
    (2717, 2754, "lateral: AP roll"),
    (2755, 3066, "ALT_ACQ (capture law, speed protection)"),
    (3067, 3107, "ALT_HOLD"),
    (3108, 3186, "FLARE V/S law"),
    (3187, 3322, "G/S"),
    (3323, 3473, "FPA"),
    (3474, 3556, "G/S / path laws (between FPA and SPD_MACH)"),
    (3557, 3655, "SPD_MACH"),
    (3656, 3791, "SPD_MACH speed laws"),
    (3792, 3955, "V/S"),
    (3956, 4069, "SRS"),
    (4070, 4094, "VPATH"),
    (4095, 4169, "FD output switch"),
    (4170, 4481, "flare law, AP side of SPD/SRS, VS limiters"),
    (4482, 4559, "AP output"),
    (4560, 99999, "flare condition, delays, constructor"),
]


def subsystem(table, line):
    for lo, hi, name in table:
        if lo <= line <= hi:
            return name
    return "?"


def load(path):
    with open(path) as f:
        return json.load(f)["data"][0]


def branch_key(fname, b):
    return (os.path.basename(fname), b[0], b[1], b[2], b[3])


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("export")
    ap.add_argument("--baseline")
    ap.add_argument("--source-dir", default=None)
    ap.add_argument("--max-lines", type=int, default=0, help="max uncovered branches listed per subsystem (0 = all)")
    args = ap.parse_args()

    data = load(args.export)
    base = None
    if args.baseline:
        base = {}
        for f in load(args.baseline)["files"]:
            for b in f.get("branches", []):
                base[branch_key(f["filename"], b)] = (b[4], b[5])

    print("== coverage per file")
    for f in data["files"]:
        s = f["summary"]
        name = os.path.basename(f["filename"])
        print(f"{name:24s} lines {s['lines']['covered']:6d}/{s['lines']['count']:6d} ({s['lines']['percent']:5.1f} %)"
              f"  regions {s['regions']['covered']:6d}/{s['regions']['count']:6d} ({s['regions']['percent']:5.1f} %)"
              f"  branches {s['branches']['covered']:6d}/{s['branches']['count']:6d} ({s['branches']['percent']:5.1f} %)")

    for f in data["files"]:
        name = os.path.basename(f["filename"])
        table = COMPUTER if name == "FmgcComputer.cpp" else OUTER if name == "FmgcOuterLoops.cpp" else None
        if table is None:
            continue
        src = None
        sd = args.source_dir
        path = os.path.join(sd, name) if sd else f["filename"]
        if os.path.exists(path):
            with open(path) as fh:
                src = fh.read().split("\n")
        groups = collections.OrderedDict((n, []) for _, _, n in table)
        totals = collections.defaultdict(lambda: [0, 0, 0, 0])  # conditions, both sides, one side, never
        gained = collections.defaultdict(int)
        seen = set()
        for b in f.get("branches", []):
            key = branch_key(f["filename"], b)
            if key in seen:
                continue  # expansions can repeat a branch
            seen.add(key)
            line, t, fl = b[0], b[4], b[5]
            g = subsystem(table, line)
            tot = totals[g]
            tot[0] += 1
            if t > 0 and fl > 0:
                tot[1] += 1
            elif t > 0 or fl > 0:
                tot[2] += 1
            else:
                tot[3] += 1
            if base is not None and key in base:
                bt, bf = base[key]
                gained[g] += (t > 0 and bt == 0) + (fl > 0 and bf == 0)
            if t == 0 or fl == 0:
                text = ""
                if src and 0 < b[0] <= len(src):
                    if b[0] == b[2]:
                        text = src[b[0] - 1][b[1] - 1:b[3] - 1]
                    else:
                        text = src[b[0] - 1][b[1] - 1:] + " ..."
                side = "never evaluated" if t == 0 and fl == 0 else ("never true" if t == 0 else "never false")
                groups[g].append((line, b[1], side, t + fl, text.strip()))
        print(f"\n== {name}: branch conditions per subsystem (conditions / both sides / one side only / never "
              f"evaluated{' / sides gained vs baseline' if base is not None else ''})")
        for _, _, g in table:
            tot = totals[g]
            if tot[0] == 0:
                continue
            extra = f"  +{gained[g]}" if base is not None else ""
            print(f"  {g:72s} {tot[0]:5d} {tot[1]:5d} {tot[2]:5d} {tot[3]:5d}{extra}")
        print(f"\n== {name}: uncovered branch sides")
        for g, items in groups.items():
            if not items:
                continue
            print(f"\n-- {g} ({len(items)})")
            lim = items if args.max_lines <= 0 else items[: args.max_lines]
            for line, col, side, n, text in lim:
                print(f"  {line:5d}:{col:<3d} {side:16s} (evaluated {n:>9d})  {text}")
            if len(lim) < len(items):
                print(f"  ... {len(items) - len(lim)} more")


if __name__ == "__main__":
    sys.exit(main())
