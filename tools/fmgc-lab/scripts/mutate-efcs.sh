#!/bin/bash
# Mutation test of the ELAC or SEC differential check (scripts/mutate-fcu.py with UNIT=elac (default) or UNIT=sec): list the mutants of
# readable/<unit>/, build one <unit>-check binary per mutant in the dev container (needs a configured build/cmake:
# scripts/build.sh), then run them on the host, JOBS (default 4) at a time. Results in build/mutants-${UNIT:-elac}/results.txt.
# The sources are restored after each mutant and the unmutated <unit>-check-stock is rebuilt at the end.
# MUTANTS=12,34,...: rebuild and retest only those mutants (indices of build/mutants-${UNIT:-elac}/list.json).
set -euo pipefail
here="$(cd "$(dirname "$0")/.." && pwd)"
repo="$(cd "$here/../.." && pwd)"
export UNIT=${UNIT:-elac}
if [ -z "${MUTANTS:-}" ]; then
  rm -rf "$here/build/mutants-${UNIT:-elac}"
fi
python3 "$here/scripts/mutate-fcu.py" list
(cd "$repo" && ./scripts/dev-env/run.sh bash -c "cd tools/fmgc-lab && UNIT=${UNIT:-elac} BUILD_JOBS=${BUILD_JOBS:-2} MUTANTS='${MUTANTS:-}' python3 \
  scripts/mutate-fcu.py build && chown -R $(id -u):$(id -g) build/mutants-${UNIT:-elac}")
git -C "$repo" diff --quiet -- tools/fmgc-lab/readable/${UNIT:-elac} || { echo "error: readable/${UNIT:-elac} not restored" >&2; exit 2; }
python3 "$here/scripts/mutate-fcu.py" test "${JOBS:-4}"
