#!/usr/bin/env bash
#
# Warm the Gradle/Android caches online so an offline install will work later.
#
# Run this while you have a network connection. It does two things:
#   1. Online build of the TeamCode debug APK — pulls the Gradle distribution,
#      every dependency artifact, and any missing Android SDK build-tools into
#      the local caches.
#   2. Offline verification — `clean` then rebuild with `--offline`, forcing a
#      full recompile/repackage using only what's now cached. If this succeeds,
#      you have real assurance that a later offline `./deploy.sh` (which runs
#      installDebug) won't need the network for dependencies.
#
# Usage:
#   ./prefetch.sh              # warm caches, then verify the offline build
#   ./prefetch.sh --no-verify  # warm caches only, skip the offline rebuild
set -euo pipefail

DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
TASK=":TeamCode:assembleDebug"

VERIFY=1
for arg in "$@"; do
  case "$arg" in
    --no-verify) VERIFY=0 ;;
    *) echo "error: unknown argument '$arg'" >&2; exit 2 ;;
  esac
done

gw() { "$DIR/gradlew" -p "$DIR" "$@"; }

echo "==> Warming caches with an online build of $TASK ..."
# --refresh-dependencies makes Gradle re-resolve every dependency, so anything
# missing from the cache is fetched now rather than silently relied on later.
gw --refresh-dependencies "$TASK"

if [[ "$VERIFY" -eq 0 ]]; then
  echo "Caches warmed; skipping offline verification (--no-verify)."
  exit 0
fi

echo "==> Verifying: clean + offline rebuild of $TASK (no network) ..."
# clean forces real compilation/packaging rather than up-to-date no-ops, so the
# offline run genuinely exercises the cached dependencies.
if gw --offline clean "$TASK"; then
  echo
  echo "OK: offline build succeeded. Dependencies are cached — you can install"
  echo "    offline with ./deploy.sh (or: ./gradlew --offline :TeamCode:installDebug)."
else
  echo
  echo "FAIL: offline build failed — something still needs the network." >&2
  echo "      Re-run ./prefetch.sh while online to fetch the missing pieces." >&2
  exit 1
fi
