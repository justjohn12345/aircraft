#!/bin/bash
# Mutation test of the FCU differential check (scripts/mutate-fcu.py): list the mutants of readable/fcu/, build one
# fcu-check binary per mutant in the dev container (needs a configured build/cmake: scripts/build.sh), then run them on
# the host, JOBS (default 4) at a time. Results in build/mutants/results.txt. The sources are restored after each
# mutant and the unmutated fcu-check-stock is rebuilt at the end.
# MUTANTS=139,169,...: rebuild and retest only those mutants (indices of build/mutants/list.json; the list must not
# have changed, i.e. readable/fcu unchanged since the full run), e.g. the survivors after adding a scenario.
set -euo pipefail
here="$(cd "$(dirname "$0")/.." && pwd)"
repo="$(cd "$here/../.." && pwd)"
if [ -z "${MUTANTS:-}" ]; then
  rm -rf "$here/build/mutants"
fi
python3 "$here/scripts/mutate-fcu.py" list
(cd "$repo" && ./scripts/dev-env/run.sh bash -c "cd tools/fmgc-lab && MUTANTS='${MUTANTS:-}' python3 scripts/mutate-fcu.py \
  build && chown -R $(id -u):$(id -g) build/mutants")
git -C "$repo" diff --quiet -- tools/fmgc-lab/readable/fcu || { echo "error: readable/fcu not restored" >&2; exit 2; }
python3 "$here/scripts/mutate-fcu.py" test "${JOBS:-4}"
