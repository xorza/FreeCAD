#!/bin/bash
# macOS: fetch upstream releases/FreeCAD-1-1, merge into personal/1.1-patched,
# do a clean release build, and refresh the local .app launcher.
set -euo pipefail

REPO="$(cd "$(dirname "$0")" && pwd)"
cd "$REPO"

UPSTREAM_REMOTE="upstream"
UPSTREAM_URL="https://github.com/FreeCAD/FreeCAD.git"
UPSTREAM_BRANCH="releases/FreeCAD-1-1"
LOCAL_BRANCH="personal/1.1-patched"
APP="$REPO/dist/FreeCAD.app"
BINARY="$REPO/build/release/bin/FreeCAD"
ICON_SRC="$REPO/src/MacAppBundle/FreeCAD.app/Contents/Resources/freecad.icns"

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

echo "==> Wiping build/release for a clean build"
rm -rf build/release

echo "==> Configuring release"
pixi run configure-release

echo "==> Building release"
pixi run build-release

LSREGISTER="/System/Library/Frameworks/CoreServices.framework/Frameworks/LaunchServices.framework/Support/lsregister"
APPLICATIONS_APP="$HOME/Applications/FreeCAD.app"

if [[ ! -x "$BINARY" ]]; then
    echo "==> $BINARY missing after build; aborting" >&2
    exit 1
fi

echo "==> Generating launcher bundle at $APP"
rm -rf "$APP"
mkdir -p "$APP/Contents/MacOS" "$APP/Contents/Resources"
cp "$ICON_SRC" "$APP/Contents/Resources/freecad.icns"
cat > "$APP/Contents/Info.plist" <<EOF
<?xml version="1.0" encoding="UTF-8"?>
<!DOCTYPE plist PUBLIC "-//Apple//DTD PLIST 1.0//EN" "http://www.apple.com/DTDs/PropertyList-1.0.dtd">
<plist version="1.0">
<dict>
    <key>CFBundleExecutable</key><string>FreeCAD</string>
    <key>CFBundleIdentifier</key><string>org.freecad.FreeCAD.dev</string>
    <key>CFBundleName</key><string>FreeCAD</string>
    <key>CFBundleDisplayName</key><string>FreeCAD</string>
    <key>CFBundlePackageType</key><string>APPL</string>
    <key>CFBundleShortVersionString</key><string>1.1</string>
    <key>CFBundleVersion</key><string>1.1</string>
    <key>CFBundleIconFile</key><string>freecad.icns</string>
    <key>NSHighResolutionCapable</key><true/>
    <key>LSMinimumSystemVersion</key><string>11.0</string>
</dict>
</plist>
EOF
# LaunchServices rejects shell-script main executables (error -10669),
# so compile a tiny native launcher that execs the build binary.
LAUNCHER_C="$(mktemp -t freecad-launcher).c"
cat > "$LAUNCHER_C" <<EOF
#include <unistd.h>
#include <stdio.h>
int main(int argc, char *argv[]) {
    argv[0] = (char *)"$BINARY";
    execv("$BINARY", argv);
    perror("execv");
    return 1;
}
EOF
clang -arch arm64 -o "$APP/Contents/MacOS/FreeCAD" "$LAUNCHER_C"
rm -f "$LAUNCHER_C"

echo "==> Signing and registering $APP"
codesign --force --sign - "$APP"
touch "$APP"
"$LSREGISTER" -f "$APP"

echo "==> Copying to $APPLICATIONS_APP"
mkdir -p "$HOME/Applications"
rm -rf "$APPLICATIONS_APP"
cp -R "$APP" "$APPLICATIONS_APP"
"$LSREGISTER" -f "$APPLICATIONS_APP"

echo "==> Done. Launch via Spotlight or: open '$APPLICATIONS_APP'"
