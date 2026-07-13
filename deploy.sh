#!/usr/bin/env bash
#
# Make sure we're talking to the robot over adb, then build & install TeamCode.
#
# Confirms an adb device is present; if the connection dropped (common over
# Wi-Fi as the hub sleeps or roams), it retries `adb connect` a few times before
# giving up. Once connected it runs the Gradle install of the TeamCode app.
#
# Connection: handled by ensure-robot-connected.sh — uses a USB device if present,
# else connects to the Control Hub AP (192.168.43.1:5555 by default; override with
# ADB_HOST). Set ADB_SERIAL to target a specific device when several are attached.
#
# Usage:
#   ./deploy.sh                 # ensure connection, then installDebug
#   ./deploy.sh --no-install    # just (re)establish the adb connection
#   ADB_HOST=192.168.43.1 ./deploy.sh
set -euo pipefail

DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
GRADLE_TASK=":TeamCode:installDebug"

INSTALL=1
for arg in "$@"; do
  case "$arg" in
    --no-install) INSTALL=0 ;;
    *) echo "error: unknown argument '$arg'" >&2; exit 2 ;;
  esac
done

source "$DIR/ensure-robot-connected.sh"
trap disconnect_robot EXIT
ensure_robot_connected

if [[ "$INSTALL" -eq 0 ]]; then
  echo "Connection ready; skipping install (--no-install)."
  exit 0
fi

echo "Installing $GRADLE_TASK ..."
"$DIR/gradlew" -p "$DIR" "$GRADLE_TASK"
echo "Done."
