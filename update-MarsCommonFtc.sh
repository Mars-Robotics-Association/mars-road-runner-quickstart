#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

echo "Fetching tags from MarsCommonFtc..."
cd "$SCRIPT_DIR/MarsCommonFtc"
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
    echo "No tags found in MarsCommonFtc."
    exit 1
fi

echo ""
echo "Available MarsCommonFtc tags (newest first):"
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
echo ""
echo "Checking out tag: $SELECTED_TAG"

git checkout "tags/$SELECTED_TAG" --quiet

echo "Updating nested submodules..."
git submodule update --init --recursive
cd "$SCRIPT_DIR"

echo ""
echo "MarsCommonFtc updated to $SELECTED_TAG."
