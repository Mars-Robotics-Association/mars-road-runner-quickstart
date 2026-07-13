#!/usr/bin/env bash
#
# Shared adb connection helper for the robot tooling (deploy.sh). Source it, then
# call `ensure_robot_connected`:
#
#   DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
#   source "$DIR/ensure-robot-connected.sh"   # adjust path as needed
#   ensure_robot_connected
#   adbc shell ...                            # adbc wrapper is now available
#
# It can also be run directly to just (re)establish the connection:
#   ./ensure-robot-connected.sh
#
# Behavior: if an adb device is already present (e.g. USB), it's used as-is.
# Otherwise it tries `adb connect` to the hub, retrying a few times. The target
# defaults to the Control Hub AP at 192.168.43.1:5555; override with ADB_HOST.
# Set ADB_SERIAL to pin a specific device when several are attached.

# Default target: the Control Hub's own Wi-Fi AP. Override via ADB_HOST.
ROBOT_HOST="${ADB_HOST:-192.168.43.1}"
[[ "$ROBOT_HOST" == *:* ]] || ROBOT_HOST="$ROBOT_HOST:5555"

ROBOT_CONNECT_RETRIES="${ROBOT_CONNECT_RETRIES:-5}"
ROBOT_CONNECT_WAIT="${ROBOT_CONNECT_WAIT:-3}"

# adb wrapper that honors an optional explicit serial.
adbc() {
  if [[ -n "${ADB_SERIAL:-}" ]]; then
    adb -s "$ADB_SERIAL" "$@"
  else
    adb "$@"
  fi
}

# True when adb reports a usable (device) state for our target.
_robot_is_connected() {
  [[ "$(adbc get-state 2>/dev/null)" == "device" ]]
}

# Ensure an adb device is reachable, connecting to the hub over Wi-Fi if needed.
# Returns non-zero (and prints to stderr) if it can't establish a connection.
ensure_robot_connected() {
  if ! command -v adb >/dev/null 2>&1; then
    echo "error: adb not found (add Android platform-tools to PATH)" >&2
    return 1
  fi

  if _robot_is_connected; then
    echo "adb: connected (${ADB_SERIAL:-usb})"
    return 0
  fi

  local i
  for ((i = 1; i <= ROBOT_CONNECT_RETRIES; i++)); do
    echo "adb: not connected — connect attempt $i/$ROBOT_CONNECT_RETRIES to $ROBOT_HOST ..."
    # Drop any stale offline entry before retrying.
    adb disconnect "$ROBOT_HOST" >/dev/null 2>&1 || true
    adb connect "$ROBOT_HOST" >/dev/null 2>&1 || true
    # Pin the wrapper to the host we just connected, so it isn't ambiguous when a
    # USB device is also attached.
    ADB_SERIAL="${ADB_SERIAL:-$ROBOT_HOST}"
    if _robot_is_connected; then
      echo "adb: connected to $ROBOT_HOST"
      # Remember that we (not USB) opened this network link, so disconnect_robot
      # only tears down a connection we actually established.
      _ROBOT_NETWORK_CONNECTED=1
      return 0
    fi
    sleep "$ROBOT_CONNECT_WAIT"
  done

  echo "error: could not reach the hub at $ROBOT_HOST after $ROBOT_CONNECT_RETRIES attempts." >&2
  echo "       Check the robot is on, you're on its Wi-Fi, and adb is enabled." >&2
  return 1
}

# Tear down a network connection that ensure_robot_connected opened. No-op for a
# USB session (we never connected over the network), so it's safe as an EXIT trap.
# Intended use:  trap disconnect_robot EXIT
disconnect_robot() {
  if [[ "${_ROBOT_NETWORK_CONNECTED:-0}" -eq 1 ]]; then
    echo "adb: disconnecting $ROBOT_HOST"
    adb disconnect "$ROBOT_HOST" >/dev/null 2>&1 || true
    _ROBOT_NETWORK_CONNECTED=0
  fi
}

# When executed directly (not sourced), just establish the connection.
if ! (return 0 2>/dev/null); then
  set -euo pipefail
  ensure_robot_connected
fi
