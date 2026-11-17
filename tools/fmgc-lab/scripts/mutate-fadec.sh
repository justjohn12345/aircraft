#!/bin/bash
# Mutation test of the FADEC differential check (scripts/mutate-fcu.py with UNIT=fadec): list the mutants of
# readable/fadec/, build one fadec-check binary per mutant in the dev container (needs a configured build/cmake:
# scripts/build.sh), then run them on the host, JOBS (default 4) at a time. Results in build/mutants-fadec/results.txt.
# The sources are restored after each mutant and the unmutated fadec-check-stock is rebuilt at the end.
# MUTANTS=12,34,...: rebuild and retest only those mutants (indices of build/mutants-fadec/list.json).
set -euo pipefail
here="$(cd "$(dirname "$0")/.." && pwd)"
repo="$(cd "$here/../.." && pwd)"
export UNIT=fadec
if [ -z "${MUTANTS:-}" ]; then
  rm -rf "$here/build/mutants-fadec"
fi
python3 "$here/scripts/mutate-fcu.py" list
(cd "$repo" && ./scripts/dev-env/run.sh bash -c "cd tools/fmgc-lab && UNIT=fadec MUTANTS='${MUTANTS:-}' python3 \
  scripts/mutate-fcu.py build && chown -R $(id -u):$(id -g) build/mutants-fadec")
git -C "$repo" diff --quiet -- tools/fmgc-lab/readable/fadec || { echo "error: readable/fadec not restored" >&2; exit 2; }
python3 "$here/scripts/mutate-fcu.py" test "${JOBS:-4}"
