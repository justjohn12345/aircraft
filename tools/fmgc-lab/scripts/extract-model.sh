#!/bin/bash
# Extracts the generated A32NX model sources at a git ref into build/models/<name>.
# Usage: scripts/extract-model.sh <git-ref> <name>
set -euo pipefail
cd "$(dirname "$0")/../../.."
ref=$1
name=$2
out=tools/fmgc-lab/build/models/$name
mkdir -p "$out"
git archive "$ref" fbw-a32nx/src/wasm/fbw_a320/src/model | tar -x -C "$out" --strip-components=6
echo "$ref $(git rev-parse --short "$ref")" > "$out/REF"
