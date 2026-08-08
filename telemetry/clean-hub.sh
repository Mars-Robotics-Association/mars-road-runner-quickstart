#!/usr/bin/env bash
#
# Delete telemetry CSVs off the Control Hub once we have a compressed copy.
#
# For every *.csv in FIRST/ on the hub, if a matching <name>.br already exists in
# telemetry/, the hub copy is redundant and gets removed. CSVs we don't yet have
# compressed locally are left alone (run pull.sh first to grab those).
#
# Connection: handled by ensure-robot-connected.sh — uses a USB device if present,
# else connects to the Control Hub AP (192.168.43.1:5555 by default; override with
# ADB_HOST). Set ADB_SERIAL to target a specific device when several are attached.
#
# Usage:
#   telemetry/clean-hub.sh             # delete redundant hub CSVs
#   telemetry/clean-hub.sh --dry-run   # show what would be deleted, delete nothing
set -euo pipefail

DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REMOTE_DIR="/sdcard/FIRST"

DRY_RUN=0
for arg in "$@"; do
  case "$arg" in
    --dry-run) DRY_RUN=1 ;;
    *) echo "error: unknown argument '$arg'" >&2; exit 2 ;;
  esac
done

source "$DIR/../ensure-robot-connected.sh"
trap disconnect_robot EXIT
ensure_robot_connected

# List remote CSVs. `ls -1` so each lands on its own line; tolerate an empty dir.
remote_list="$(adbc shell "ls -1 $REMOTE_DIR/*.csv 2>/dev/null" | tr -d '\r')"
if [[ -z "$remote_list" ]]; then
  echo "No *.csv files in $REMOTE_DIR on the hub — nothing to do."
  exit 0
fi

# Decide what to delete first, then delete in one batched adb call. Calling adb
# inside the read loop would let `adb shell` swallow the rest of the here-string
# on stdin, so only the first file ever got processed.
kept=0
to_delete=()           # remote paths
quoted_targets=()      # shell-quoted remote paths for a single rm
while IFS= read -r remote; do
  [[ -n "$remote" ]] || continue
  name="$(basename "$remote")"
  if [[ -f "$DIR/$name.br" ]]; then
    if [[ "$DRY_RUN" -eq 1 ]]; then
      echo "would delete  $name (have $name.br)"
    else
      echo "delete  $name (have $name.br)"
    fi
    to_delete+=("$remote")
    quoted_targets+=("'$remote'")
  else
    echo "keep    $name (no local .br — run pull.sh)"
    kept=$((kept + 1))
  fi
done <<< "$remote_list"

deleted="${#to_delete[@]}"
if [[ "$DRY_RUN" -eq 1 ]]; then
  echo "Dry run: $deleted would be deleted, $kept kept."
else
  if [[ "$deleted" -gt 0 ]]; then
    # One rm for all targets; </dev/null so adb doesn't read our stdin.
    adbc shell "rm -f ${quoted_targets[*]}" </dev/null
  fi
  echo "Done: $deleted deleted, $kept kept."
fi
