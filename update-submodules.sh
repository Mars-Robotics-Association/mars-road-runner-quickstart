#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# Collect submodule paths from .gitmodules
SUBMODULES=()
while IFS= read -r path; do
    SUBMODULES+=("$path")
done < <(git config --file "$SCRIPT_DIR/.gitmodules" --get-regexp 'submodule\..*\.path' | awk '{print $2}')

if [[ ${#SUBMODULES[@]} -eq 0 ]]; then
    echo "No submodules found in .gitmodules."
    exit 1
fi

SUB_PATH=""
if [[ $# -ge 1 ]]; then
    REQUESTED="$1"
    for path in "${SUBMODULES[@]}"; do
        if [[ "$path" == "$REQUESTED" || "$(basename "$path")" == "$REQUESTED" ]]; then
            SUB_PATH="$path"
            break
        fi
    done
    if [[ -z "$SUB_PATH" ]]; then
        echo "ERROR: No submodule matching \"$REQUESTED\" found."
        echo "Available submodules:"
        for path in "${SUBMODULES[@]}"; do
            echo "  - $path"
        done
        exit 1
    fi
else
    echo "Available submodules:"
    echo "-----------------------------------------------"
    for i in "${!SUBMODULES[@]}"; do
        NUM=$((i + 1))
        printf "  %d) %s\n" "$NUM" "${SUBMODULES[$i]}"
    done
    echo ""
    while true; do
        read -rp "Select a submodule to update (1-${#SUBMODULES[@]}): " CHOICE
        if [[ "$CHOICE" =~ ^[0-9]+$ ]] && [[ $CHOICE -ge 1 ]] && [[ $CHOICE -le ${#SUBMODULES[@]} ]]; then
            break
        fi
        echo "Invalid selection. Please enter a number between 1 and ${#SUBMODULES[@]}."
    done
    SUB_PATH="${SUBMODULES[$((CHOICE - 1))]}"
fi

echo ""
echo "Fetching tags from $SUB_PATH..."
cd "$SCRIPT_DIR/$SUB_PATH"
if ! git fetch --prune --force --quiet origin "+refs/tags/*:refs/tags/*"; then
    echo "ERROR: Failed to fetch tags from remote \"origin\"."
    echo "       Check network access, git auth, and safe.directory settings."
    exit 1
fi

# Detect which tag is currently checked out (empty if HEAD is not on a tag)
CURRENT_TAG="$(git describe --tags --exact-match HEAD 2>/dev/null || true)"

# Collect the 10 most recent tags with their commit dates
TAGS=()
while IFS= read -r tag; do
    TAGS+=("$tag")
done < <(git tag --sort=-creatordate | head -10)
COUNT=${#TAGS[@]}

if [[ $COUNT -eq 0 ]]; then
    echo "No tags found in $SUB_PATH."
    exit 1
fi

echo ""
echo "Available $SUB_PATH tags (newest first):"
echo "-----------------------------------------------"
for i in "${!TAGS[@]}"; do
    TAG="${TAGS[$i]}"
    DATE="$(git log -1 --format="%ci" "$TAG" 2>/dev/null)"
    NUM=$((i + 1))
    if [[ "$TAG" == "$CURRENT_TAG" ]]; then
        printf "  %d) %-40s %s  <-- current\n" "$NUM" "$TAG" "$DATE"
    else
        printf "  %d) %-40s %s\n" "$NUM" "$TAG" "$DATE"
    fi
done
echo ""

while true; do
    read -rp "Enter number (1-$COUNT): " CHOICE
    if [[ "$CHOICE" =~ ^[0-9]+$ ]] && [[ $CHOICE -ge 1 ]] && [[ $CHOICE -le $COUNT ]]; then
        break
    fi
    echo "Invalid selection. Please enter a number between 1 and $COUNT."
done

SELECTED_TAG="${TAGS[$((CHOICE - 1))]}"
TARGET_SHA="$(git rev-parse "tags/${SELECTED_TAG}^{commit}")"

echo ""
echo "Checking out $SELECTED_TAG ($TARGET_SHA) ..."
# --force resets the worktree to the tag so dirty files cannot leave sources
# off the selected pin. Nested update --force does the same one level down.
git checkout --force "$TARGET_SHA"

echo "Updating nested submodules to this pin's recorded hashes..."
git submodule update --init --recursive --force
cd "$SCRIPT_DIR"

echo "Staging pointer update in parent repo..."
git add "$SUB_PATH"

# Parent-side update uses the *index* pin we just staged — ensures this path
# (and its nested submodules) really match that hash, not only the in-tree
# checkout that preceded the stage.
echo "Syncing parent view of $SUB_PATH to the staged pin..."
git submodule update --init --recursive --force -- "$SUB_PATH"

HEAD_SHA="$(git -C "$SCRIPT_DIR/$SUB_PATH" rev-parse HEAD)"
INDEX_SHA="$(git -C "$SCRIPT_DIR" ls-files -s -- "$SUB_PATH" | awk '{print $2; exit}')"
if [[ "$HEAD_SHA" != "$TARGET_SHA" ]]; then
    echo "ERROR: $SUB_PATH HEAD is $HEAD_SHA, expected $TARGET_SHA." >&2
    exit 1
fi
if [[ "$INDEX_SHA" != "$TARGET_SHA" ]]; then
    echo "ERROR: parent index pin for $SUB_PATH is $INDEX_SHA, expected $TARGET_SHA." >&2
    exit 1
fi

echo ""
echo "$SUB_PATH is at $SELECTED_TAG ($TARGET_SHA)."
echo "Worktree and nested submodules match that hash; parent pointer is staged."
echo "Review and commit when ready."
