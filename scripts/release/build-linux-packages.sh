#!/usr/bin/env bash
# SPDX-License-Identifier: GPL-2.0-or-later
set -euo pipefail

root="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
dist="${DIST_DIR:-$root/dist}"
jobs="${JOBS:-$(nproc)}"
export SOURCE_DATE_EPOCH="${SOURCE_DATE_EPOCH:-$(git -C "$root" show -s --format=%ct HEAD)}"
export CBANG_HOME="${CBANG_HOME:-$root/build-deps/cbang}"

version="$(python3 -c 'import json, sys; print(json.load(open(sys.argv[1]))["version"])' "$root/package.json")"
IFS=. read -r year month patch <<<"$version"
release_tag="${RELEASE_TAG:-$(printf 'v%04d.%02d.%d' "$year" "$month" "$patch")}"

mkdir -p "$dist"
cd "$root"
scons -c with_gui=1 with_tpl=0
scons -j"$jobs" with_gui=1 with_tpl=0
RELEASE_TAG="$release_tag" APPIMAGE_OUTPUT="$dist/CAMotics-Fast-$release_tag-linux-x86_64.AppImage" \
  scripts/build-appimage

DEB_BUILD_OPTIONS="parallel=$jobs" dpkg-buildpackage -us -uc -b
deb_source="$(find "$root/.." -maxdepth 1 -type f -name 'camotics-fast_*_amd64.deb' -print -quit)"
if [[ -z "$deb_source" ]]; then
  echo "Debian package was not produced" >&2
  exit 1
fi
install -m 0644 "$deb_source" "$dist/camotics-fast_${year}.$(printf '%02d' "$month").${patch}_amd64.deb"

(
  cd "$dist"
  sha256sum \
    "CAMotics-Fast-$release_tag-linux-x86_64.AppImage" \
    "camotics-fast_${year}.$(printf '%02d' "$month").${patch}_amd64.deb" \
    >SHA256SUMS.linux
)
