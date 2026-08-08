#!/usr/bin/env bash
#
# Pull new robot telemetry CSVs off the Control Hub and store them compressed.
#
# Reads the FIRST/ folder on the hub (where CsvLogger writes), pulls every *.csv
# that isn't already in telemetry/ (as either .csv or .csv.br), then hands them
# to compress.sh for brotli (.csv.br) compression. Originals are left on the hub;
# run clean-hub.sh afterward to delete the ones that now have a local .csv.br.
#
# Connection: handled by ensure-robot-connected.sh — uses a USB device if present,
# else connects to the Control Hub AP (192.168.43.1:5555 by default; override with
# ADB_HOST). Set ADB_SERIAL to target a specific device when several are attached.
#
# Usage:
#   telemetry/pull.sh            # pull new CSVs, then compress them
set -euo pipefail

DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REMOTE_DIR="/sdcard/FIRST"

for arg in "$@"; do
  case "$arg" in
    *) echo "error: unknown argument '$arg'" >&2; exit 2 ;;
  esac
done

source "$DIR/../ensure-robot-connected.sh"
trap disconnect_robot EXIT
ensure_robot_connected

# List remote CSVs. `ls -1` so each lands on its own line; tolerate an empty dir.
remote_list="$(adbc shell "ls -1 $REMOTE_DIR/*.csv 2>/dev/null" | tr -d '\r')"
if [[ -z "$remote_list" ]]; then
  echo "No *.csv files in $REMOTE_DIR on the hub — nothing to pull."
  exit 0
fi

pulled=()
while IFS= read -r remote; do
  [[ -n "$remote" ]] || continue
  name="$(basename "$remote")"
  # Skip anything we already have locally, compressed or not.
  if [[ -f "$DIR/$name" || -f "$DIR/$name.br" ]]; then
    echo "skip  $name (already local)"
    continue
  fi
  echo "pull  $name"
  # </dev/null so adb pull can't swallow the rest of our here-string on stdin.
  adbc pull "$remote" "$DIR/$name" </dev/null >/dev/null
  pulled+=("$name")
done <<< "$remote_list"

if [[ "${#pulled[@]}" -eq 0 ]]; then
  echo "No new CSVs to pull."
  exit 0
fi

echo "Pulled ${#pulled[@]} file(s); compressing ..."
compress_args=()
for name in "${pulled[@]}"; do
  compress_args+=("$DIR/$name")
done
"$DIR/compress.sh" "${compress_args[@]}"
