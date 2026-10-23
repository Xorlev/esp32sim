#!/bin/sh
# The ESP32-P4 example images the goldens boot but the web page does not run (built from
# examples/*-p4 with ESP-IDF v5.5.3; each example's README says how), pinned by SHA-256 so CI and a
# fresh checkout get the same bytes. The page's P4 hello (p4-hello-*) stays committed.
#   tools/fetch-p4-firmware.sh [DIR]      default web/wasm/fw/public
# A file already in DIR with the right hash is kept, so a cached copy costs no download; any other
# is fetched from its URL and must match its hash, or the script fails.
# Images over 1 MB, and images only the tests boot, live in mikroverk/esp32sim-firmware, pinned by
# commit so a checkout keeps fetching the bytes it was tested with; P4_FIRMWARE_BASE_URL overrides
# the base.
set -e
cd "$(dirname "$0")/.."
DIR=${1:-web/wasm/fw/public}
BASE=${P4_FIRMWARE_BASE_URL:-https://raw.githubusercontent.com/mikroverk/esp32sim-firmware/e8facb5ccae4ba4fbf303cb2e0e7d94a59f7f46f/p4}
# file, SHA-256, URL
IMAGES="p4-hello-smp-bootloader.bin fc8698c1520e0d6469861e917d84568c94d17721faa93937896ae427e8f91717 $BASE/p4-hello-smp-bootloader.bin
p4-hello-smp_world.bin 9476bccf819114fe30ad55dde4d0917e7f5213df34ec6f159b81c4a84e053f8b $BASE/p4-hello-smp_world.bin
p4-sdcard-bootloader.bin 985366855e83f25f6b9399c79aa3eac01742e0950d3e6406e24db0a32bd76816 $BASE/p4-sdcard-bootloader.bin
p4-sdcard.bin 411c93b78fc0efb199e3576c510bf0de23f74086a87cc3a9204c67a207428749 $BASE/p4-sdcard.bin"

sha256_ok() {  # FILE HASH; -c without GNU-only --status, so macOS shasum and sha256sum both work
  if command -v shasum >/dev/null 2>&1; then echo "$2  $1" | shasum -a 256 -c >/dev/null 2>&1
  else echo "$2  $1" | sha256sum -c >/dev/null 2>&1; fi
}

mkdir -p "$DIR"
tmp=$(mktemp -d)
trap 'rm -rf "$tmp"' EXIT
echo "$IMAGES" | while read -r name hash url; do
  if [ -f "$DIR/$name" ] && sha256_ok "$DIR/$name" "$hash"; then continue; fi
  curl -fsSL --retry 5 --retry-all-errors --retry-delay 5 "$url" -o "$tmp/$name"
  sha256_ok "$tmp/$name" "$hash" || { echo "$url: SHA-256 mismatch" >&2; exit 1; }
  mv "$tmp/$name" "$DIR/$name"
  echo "fetched $name"
done
echo "P4 example images in $DIR: $(echo "$IMAGES" | wc -l | tr -d ' ') pinned files present"
