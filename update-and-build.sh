#!/bin/bash
# Fetch upstream releases/FreeCAD-1-1, merge into personal/1.1-patched,
# wipe CMake cache, build release, and refresh the local .app launcher.
set -euo pipefail

REPO="$(cd "$(dirname "$0")" && pwd)"
cd "$REPO"

UPSTREAM_REMOTE="upstream"
UPSTREAM_URL="https://github.com/FreeCAD/FreeCAD.git"
UPSTREAM_BRANCH="releases/FreeCAD-1-1"
LOCAL_BRANCH="personal/1.1-patched"
APP="$REPO/dist/FreeCAD.app"

echo "==> Ensuring upstream remote"
if ! git remote get-url "$UPSTREAM_REMOTE" >/dev/null 2>&1; then
    git remote add "$UPSTREAM_REMOTE" "$UPSTREAM_URL"
fi

echo "==> Fetching $UPSTREAM_REMOTE/$UPSTREAM_BRANCH (and main, used by build version helpers)"
git fetch "$UPSTREAM_REMOTE" "$UPSTREAM_BRANCH" main

echo "==> Checking out $LOCAL_BRANCH"
if ! git diff --quiet || ! git diff --cached --quiet; then
    echo "Working tree has uncommitted changes. Commit or stash first." >&2
    exit 1
fi
git checkout "$LOCAL_BRANCH"

echo "==> Merging $UPSTREAM_REMOTE/$UPSTREAM_BRANCH"
git merge --no-edit "$UPSTREAM_REMOTE/$UPSTREAM_BRANCH"

echo "==> Removing CMake cache"
rm -f build/release/CMakeCache.txt
rm -rf build/release/CMakeFiles

echo "==> Configuring release"
pixi run configure-release

echo "==> Building release"
pixi run build-release

LSREGISTER="/System/Library/Frameworks/CoreServices.framework/Frameworks/LaunchServices.framework/Support/lsregister"
APPLICATIONS_APP="$HOME/Applications/FreeCAD.app"

if [[ -d "$APP" ]]; then
    echo "==> Refreshing $APP"
    codesign --force --sign - "$APP"
    touch "$APP"
    "$LSREGISTER" -f "$APP"

    echo "==> Copying to $APPLICATIONS_APP"
    mkdir -p "$HOME/Applications"
    rm -rf "$APPLICATIONS_APP"
    cp -R "$APP" "$APPLICATIONS_APP"
    "$LSREGISTER" -f "$APPLICATIONS_APP"
else
    echo "==> $APP not found, skipping bundle refresh"
fi

echo "==> Done. Launch via Spotlight or: open '$APPLICATIONS_APP'"
