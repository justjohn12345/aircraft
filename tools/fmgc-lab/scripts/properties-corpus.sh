#!/bin/bash
# Checks the behavioural properties (src/properties.h) and the in-harness differential oracles over the replay corpus
# with the same scenario configs as scripts/coverage.sh replay, using the gcc corpus driver (build/fmgc-drive-stock,
# scripts/build-fuzz.sh). Output: build/properties/runs/*.txt and build/properties/props.txt (aggregated table, first
# reproducer per violated property: recording, flags, step, time).
# usage: [JOBS=4] [SEEDS=2] scripts/properties-corpus.sh [CORPUS_DIR]
set -euo pipefail
here="$(cd "$(dirname "$0")/.." && pwd)"
jobs=${JOBS:-4}
seeds=${SEEDS:-2}
corpus=${1:-$HOME/fdr-corpus/raw}
out="$here/build/properties"
rm -rf "$out"
mkdir -p "$out/runs"
list="$out/runs.txt"
: > "$list"
for f in "$corpus"/*.fdr; do
  echo "$f|plain|" >> "$list"
  for s in $(seq 1 "$seeds"); do
    echo "$f|fuzz$s|--fuzz $s" >> "$list"
    echo "$f|approach$s|--approach $s" >> "$list"
    echo "$f|athr$s|--athr $s --approach $s" >> "$list"
    echo "$f|lateral$s|--lateral $s --approach $s" >> "$list"
    echo "$f|engagement$s|--engagement $s --approach $s" >> "$list"
    echo "$f|targets$s|--targets $s --approach $s" >> "$list"
    echo "$f|failures$s|--failures $s --approach $s" >> "$list"
    echo "$f|tcas$s|--tcas $s --situation $s" >> "$list"
    echo "$f|all$s|--fuzz $s --approach $s --athr $s --lateral $s --engagement $s --targets $s --failures $s --tcas $s --situation $s" >> "$list"
  done
done
export out here
# shellcheck disable=SC2016
xargs -P "$jobs" -d '\n' -n 1 bash -c 'IFS="|" read -r f tag args <<< "$0"; b=$(basename "$f" .fdr); \
  "$here/build/fmgc-drive-stock" $args "$f" > "$out/runs/$b-$tag.txt" 2>&1 || echo "FAILED $b $tag"' < "$list"
python3 "$here/scripts/props_summary.py" "$out"/runs/*.txt > "$out/props.txt"
grep -h "^finding" "$out"/runs/*.txt | awk '{n[$2]+=$3} END {for (k in n) print "finding", k, n[k]}' | sort > "$out/findings.txt"
cat "$out/props.txt"
echo
echo "all findings (property ids, and differential checks by field) over all runs:"
cat "$out/findings.txt"
