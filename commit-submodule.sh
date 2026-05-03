#!/usr/bin/env bash
# Usage: ./commit-submodule.sh <tag> [commit-message]
# Commits changes in MarsCommonFtc, tags, pushes, then updates the parent repo.
set -euo pipefail

TAG="${1:-}"
MSG="${2:-}"

if [[ -z "$TAG" ]]; then
    echo "Usage: $0 <tag> [commit-message]" >&2
    exit 1
fi

SUBMODULE="MarsCommonFtc"
PARENT_DIR="$(cd "$(dirname "$0")" && pwd)"
SUB_DIR="$PARENT_DIR/$SUBMODULE"

# --- Submodule ---
cd "$SUB_DIR"

# Submodules check out in detached HEAD; get the configured tracking branch and switch to it.
BRANCH="$(git config -f "$PARENT_DIR/.gitmodules" "submodule.$SUBMODULE.branch" || true)"
if [[ -z "$BRANCH" ]]; then
    BRANCH="$(git rev-parse --abbrev-ref HEAD)"
fi
if [[ "$BRANCH" == "HEAD" ]]; then
    echo "Cannot determine branch for $SUBMODULE — set 'branch' in .gitmodules or check out a branch manually." >&2
    exit 1
fi
git checkout "$BRANCH"

if [[ -z "$(git status --porcelain)" ]]; then
    echo "No changes in $SUBMODULE to commit."
else
    if [[ -z "$MSG" ]]; then
        read -rp "Commit message for $SUBMODULE: " MSG
    fi
    git add -A
    git commit -m "$MSG"
fi

if ! git rev-parse "$TAG" >/dev/null 2>&1; then
    git tag "$TAG"
fi
git push origin "$BRANCH" --tags
echo "Pushed $SUBMODULE branch '$BRANCH' and tag '$TAG'."

# --- Parent repo ---
cd "$PARENT_DIR"
git submodule update --remote -- "$SUBMODULE"
git add "$SUBMODULE"

if [[ -z "$(git diff --cached --name-only)" ]]; then
    echo "Parent repo submodule pointer already up to date."
else
    git commit -m "Update $SUBMODULE to $TAG"
    echo "Parent repo updated to $TAG."
fi
