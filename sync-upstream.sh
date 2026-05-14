#!/usr/bin/env bash
# Fetch upstream FreeCAD 1.1, merge into current branch, reconfigure and rebuild release.
set -euo pipefail

UPSTREAM_REMOTE="upstream"
UPSTREAM_BRANCH="releases/FreeCAD-1-1"

cd "$(dirname "$(readlink -f "$0")")"

if ! git remote get-url "$UPSTREAM_REMOTE" >/dev/null 2>&1; then
    echo "error: remote '$UPSTREAM_REMOTE' not configured" >&2
    exit 1
fi

if ! git diff --quiet || ! git diff --cached --quiet; then
    echo "error: working tree is dirty — commit or stash first" >&2
    git status --short
    exit 1
fi

echo ">> fetching $UPSTREAM_REMOTE/$UPSTREAM_BRANCH"
git fetch "$UPSTREAM_REMOTE" "$UPSTREAM_BRANCH"

INCOMING=$(git log --oneline "HEAD..$UPSTREAM_REMOTE/$UPSTREAM_BRANCH")
if [ -z "$INCOMING" ]; then
    echo ">> already up to date — nothing to merge, skipping rebuild"
    exit 0
fi

echo ">> incoming commits:"
echo "$INCOMING"

echo ">> merging"
if ! git merge "$UPSTREAM_REMOTE/$UPSTREAM_BRANCH" --no-edit; then
    echo "error: merge conflict — resolve manually" >&2
    exit 1
fi

echo ">> nuking CMake cache"
rm -f build/release/CMakeCache.txt

echo ">> configuring release"
pixi run configure-release

echo ">> building release"
pixi run build-release

echo ">> done"
