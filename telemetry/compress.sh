#!/usr/bin/env bash
#
# Compress robot telemetry CSVs to brotli (.csv.br) for storage in the repo.
# Brotli -q11 gives ~12x on this numeric telemetry and decompresses fast; the
# .gitattributes diff=brotli mapping keeps `git diff`/`git show` readable.
#
# Usage:
#   telemetry/compress.sh                # compress every *.csv in telemetry/
#   telemetry/compress.sh foo.csv bar.csv  # compress the named files
#
# Each file is compressed, verified byte-for-byte against the original, and only
# then is the original removed. An existing up-to-date .csv.br is left alone.
set -euo pipefail

if ! command -v brotli >/dev/null 2>&1; then
  echo "error: brotli not found (install: brew install brotli / apt install brotli)" >&2
  exit 1
fi

DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# Build the work list: explicit args, or all *.csv in this directory.
if [[ $# -gt 0 ]]; then
  files=("$@")
else
  files=("$DIR"/*.csv)
  # Guard against the literal glob when there are no matches.
  [[ -e "${files[0]}" ]] || { echo "No *.csv files in $DIR — nothing to do."; exit 0; }
fi

for f in "${files[@]}"; do
  if [[ ! -f "$f" ]]; then
    echo "skip  $f (not a file)"; continue
  fi
  if [[ "$f" == *.br ]]; then
    echo "skip  $(basename "$f") (already compressed)"; continue
  fi
  out="$f.br"
  if [[ -f "$out" && "$out" -nt "$f" ]]; then
    echo "skip  $(basename "$f") (.csv.br already current)"; continue
  fi

  brotli -q 11 -c "$f" > "$out"
  if brotli -dc "$out" | cmp -s - "$f"; then
    o=$(wc -c < "$f" | tr -d ' '); c=$(wc -c < "$out" | tr -d ' ')
    printf "ok    %-42s %s -> %s (%sx)\n" "$(basename "$f")" \
      "$(numfmt --to=iec "$o" 2>/dev/null || echo "${o}B")" \
      "$(numfmt --to=iec "$c" 2>/dev/null || echo "${c}B")" \
      "$(awk -v o="$o" -v c="$c" 'BEGIN{printf "%.1f", o/c}')"
    rm -f "$f"
  else
    rm -f "$out"
    echo "FAIL  $(basename "$f") — verification mismatch, original kept" >&2
    exit 1
  fi
done
