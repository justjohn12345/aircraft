#!/usr/bin/env python3
# Aggregates the property tables printed by fmgc-drive (one file per run) into one table: frames checked, bad frames
# and violation episodes summed over all runs, and the first reproducer of each violated property (preferring plain
# replays, then the run with the fewest scenario flags).
#
# usage: props_summary.py RUN_OUTPUT...

import sys


def main():
    order = []
    agg = {}
    for path in sys.argv[1:]:
        fname, config = "?", ""
        in_table = False
        with open(path) as f:
            for line in f:
                if line.startswith("file "):
                    fname = line.split(None, 1)[1].strip().rsplit("/", 1)[-1]
                elif line.startswith("config"):
                    config = line[len("config"):].strip()
                elif line.startswith("property "):
                    in_table = True
                elif in_table and line.strip():
                    parts = line.split(None, 6)
                    if len(parts) < 6:
                        continue
                    pid = parts[0]
                    checked, bad, viol, first = int(parts[1]), int(parts[2]), int(parts[3]), int(parts[4])
                    tfirst = float(parts[5])
                    detail = parts[6].strip() if len(parts) > 6 else ""
                    if pid not in agg:
                        order.append(pid)
                        agg[pid] = {"checked": 0, "bad": 0, "viol": 0, "runs": 0, "repro": None}
                    a = agg[pid]
                    a["checked"] += checked
                    a["bad"] += bad
                    a["viol"] += viol
                    if viol:
                        a["runs"] += 1
                        rank = (config.count("--"), fname, tfirst)
                        if a["repro"] is None or rank < a["repro"][0]:
                            a["repro"] = (rank, f"{fname} {config or '(plain replay)'} step {first} t {tfirst:.2f}: {detail}")
    print(f"{'property':36s} {'checked':>11s} {'bad':>9s} {'violations':>10s} {'runs':>5s}  first reproducer")
    for pid in order:
        a = agg[pid]
        repro = a["repro"][1] if a["repro"] else ""
        print(f"{pid:36s} {a['checked']:11d} {a['bad']:9d} {a['viol']:10d} {a['runs']:5d}  {repro}")


if __name__ == "__main__":
    main()
