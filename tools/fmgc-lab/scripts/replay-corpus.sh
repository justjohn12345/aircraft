#!/bin/bash
# Replays every recording in the corpus through the model variant that recorded it (corpus-variants.tsv:
# "<file prefix> <variant>", first match wins, default stock). Writes one report per file to build/reports/.
# Usage: scripts/replay-corpus.sh [corpus-dir] [extra replay options]
set -uo pipefail
here="$(cd "$(dirname "$0")/.." && pwd)"
corpus=${1:-$HOME/fdr-corpus/raw}
shift || true
mkdir -p "$here/build/reports"
variant_for() {
  local f=$1
  while read -r prefix variant; do
    [[ -z "$prefix" || "$prefix" == \#* ]] && continue
    [[ "$f" == "$prefix"* ]] && { echo "$variant"; return; }
  done < "$here/corpus-variants.tsv"
  echo stock
}
for path in "$corpus"/*.fdr; do
  f=$(basename "$path" .fdr)
  v=$(variant_for "$f")
  ( "$here/build/fmgc-replay-$v" "$@" "$path" > "$here/build/reports/$f.txt" 2>&1
    rc=$?
    printf '%s\t%s\t%s\n' "$f" "$v" "$(grep -E '^(frames|gaps filled|mismatch frames)' "$here/build/reports/$f.txt" | tr -s ' ' | tr '\n' '|')" ) &
done | sort
wait
