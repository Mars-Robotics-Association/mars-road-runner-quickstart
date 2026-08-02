#!/usr/bin/env bash
#
# Warm the Gradle/Android caches online so an offline install will work later.
#
# Run this while you have a network connection. It does three things:
#   1. Ensure submodule sources match the pins in the parent *index* (committed
#      or staged-but-uncommitted). Unstaged pin drift fails fast — never reset
#      or stage. Nested/missing checkouts are filled from those pins. Offline
#      deploy has no network to fetch pins, so sources must be correct now.
#   2. Online build of the TeamCode debug APK — pulls the Gradle distribution,
#      every dependency artifact, and any missing Android SDK build-tools into
#      the local caches.
#   3. Offline verification — `clean` then rebuild with `--offline`, forcing a
#      full recompile/repackage using only what's now cached. If this succeeds,
#      you have real assurance that a later offline `./deploy.sh` (which runs
#      installDebug) won't need the network for dependencies.
#
# Usage:
#   ./prefetch.sh              # ensure pins, warm caches, verify offline
#   ./prefetch.sh --no-verify  # ensure pins + warm only; skip offline rebuild
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

echo "==> Ensuring submodules match parent index pins ..."
# Fail-fast on top-level pin drift *before* submodule update. Update would
# silently reset an unstaged intentional checkout back to the old index pin.
# Intent is captured by update-submodules (stages the gitlink) or git add.
sub_drift=0
while read -r hash path _; do
  [[ -z "${hash:-}" ]] && continue
  flag="${hash:0:1}"
  case "$flag" in
    +)
      echo "FAIL: $path is checked out at a different commit than the parent index pin." >&2
      echo "      If this pin is intentional:  git add $path" >&2
      echo "      (or re-run ./update-submodules.sh $path)" >&2
      echo "      If not:                      git submodule update --init --recursive" >&2
      sub_drift=1
      ;;
    U)
      echo "FAIL: $path has unresolved merge conflicts." >&2
      sub_drift=1
      ;;
  esac
done < <(git -C "$DIR" submodule status 2>/dev/null || true)

if [[ "$sub_drift" -ne 0 ]]; then
  exit 1
fi

# Index pins only (staged uncommitted gitlinks count). Fills missing clones and
# nested pins; does not rewrite a top-level HEAD that already matches the index.
if ! git -C "$DIR" submodule update --init --recursive; then
  echo "FAIL: could not check out submodules at the pins in the parent index." >&2
  echo "      Fix network/auth, or clean dirty submodule trees, then retry." >&2
  exit 1
fi

sub_bad=0
while read -r hash path _; do
  [[ -z "${hash:-}" ]] && continue
  flag="${hash:0:1}"
  case "$flag" in
    -)
      echo "FAIL: $path is not initialized." >&2
      sub_bad=1
      ;;
    +)
      echo "FAIL: $path is not at the commit recorded in its parent pin." >&2
      sub_bad=1
      ;;
    U)
      echo "FAIL: $path has unresolved merge conflicts." >&2
      sub_bad=1
      ;;
  esac
done < <(git -C "$DIR" submodule status --recursive 2>/dev/null || true)

if [[ "$sub_bad" -ne 0 ]]; then
  echo "       Run: git submodule update --init --recursive" >&2
  exit 1
fi

# Dirty files on the correct commit still change what gets compiled (same as
# uncommitted parent sources). Warn and ask before continuing so a dirty pin is
# an intentional choice, not a silent mismatch.
if ! git -C "$DIR" submodule foreach --recursive \
    'git diff --quiet && git diff --cached --quiet' >/dev/null 2>&1; then
  echo "WARNING: one or more submodules have uncommitted local changes." >&2
  echo "         Prefetch builds the working tree; the APK will include those edits" >&2
  echo "         and will not match the clean pin commit(s)." >&2
  git -C "$DIR" submodule foreach --recursive 'git status -s' || true
  echo >&2
  if [[ ! -t 0 ]]; then
    echo "FAIL: non-interactive session - cannot confirm dirty submodules." >&2
    echo "      Commit/stash/discard, or re-run from a terminal." >&2
    exit 1
  fi
  read -r -p "Continue with dirty submodules? [Y/N] " DIRTY_CHOICE
  case "$DIRTY_CHOICE" in
    [Yy]) echo "Continuing with dirty submodules ..." ;;
    *)
      echo "Aborted. Commit, stash, or discard submodule changes, then retry." >&2
      exit 1
      ;;
  esac
fi

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
