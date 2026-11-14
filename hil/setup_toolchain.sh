#!/bin/bash
# Install the two things the firmware needs and the repo does not carry: PlatformIO (to compile the ESP32-S3
# image) and Espressif's QEMU fork (to run it against the MuJoCo plant in hil/run_hil.py). Both go OUTSIDE the
# repo and outside the project venv, because PlatformIO pulls a dependency set of its own and the project venv
# holds a pinned mujoco (ml/requirements.txt) that must not be disturbed.
#
#   hil/setup_toolchain.sh          # install both, then print the env to use
#
# Afterwards:
#   export PATH="$HOME/.pio-venv/bin:$PATH"
#   export QEMU="$HOME/.qemu-esp/qemu/bin/qemu-system-xtensa"
#   hil/build_flash.sh && .venv/bin/python hil/run_hil.py
set -e

PIO_VENV="$HOME/.pio-venv"
QEMU_DIR="$HOME/.qemu-esp"
QEMU_REL="esp-develop-9.2.2-20261109"          # pinned: the dt_ms_* figures in hil/results*.json are
QEMU_TAG="esp_develop_9.2.2_20261109"          # specific to the QEMU build, see run_hil.py qemu_version()

# ---------------------------------------------------------------- PlatformIO
if [ ! -x "$PIO_VENV/bin/pio" ]; then
  PY=$(command -v python3.13 || command -v python3)
  echo "== PlatformIO into $PIO_VENV (with $PY)"
  "$PY" -m venv "$PIO_VENV"
  "$PIO_VENV/bin/pip" -q install --upgrade pip
  "$PIO_VENV/bin/pip" -q install platformio
fi
"$PIO_VENV/bin/pio" --version

# ---------------------------------------------------------------- Espressif QEMU (xtensa)
case "$(uname -s)-$(uname -m)" in
  Darwin-arm64)  HOST=aarch64-apple-darwin ;;
  Darwin-x86_64) HOST=x86_64-apple-darwin ;;
  Linux-aarch64) HOST=aarch64-linux-gnu ;;
  Linux-x86_64)  HOST=x86_64-linux-gnu ;;
  *) echo "no Espressif QEMU build published for $(uname -s)-$(uname -m)"; exit 1 ;;
esac
if [ ! -x "$QEMU_DIR/qemu/bin/qemu-system-xtensa" ]; then
  echo "== Espressif QEMU $QEMU_TAG ($HOST) into $QEMU_DIR"
  mkdir -p "$QEMU_DIR"; cd "$QEMU_DIR"
  BASE="https://github.com/espressif/qemu/releases/download/$QEMU_REL"
  F="qemu-xtensa-softmmu-$QEMU_TAG-$HOST.tar.xz"
  curl -fsSL -o "$F" "$BASE/$F"
  curl -fsSL -o checksum.sha256 "$BASE/qemu-$QEMU_TAG-checksum.sha256"
  # the published checksum file lists every host build; check ours against it
  WANT=$(grep -F "$F" checksum.sha256 | awk '{print $1}' | grep -E '^[0-9a-f]{64}$')
  GOT=$(shasum -a 256 "$F" | awk '{print $1}')
  [ -n "$WANT" ] && [ "$WANT" = "$GOT" ] || { echo "checksum mismatch for $F"; exit 1; }
  tar xf "$F"
fi
# It links against SDL2 even with -nographic, so the library has to be there or dyld refuses to start it.
if ! "$QEMU_DIR/qemu/bin/qemu-system-xtensa" --version >/dev/null 2>&1; then
  echo "qemu-system-xtensa will not start. Missing libraries:"
  for l in $(otool -L "$QEMU_DIR/qemu/bin/qemu-system-xtensa" 2>/dev/null | awk '/^\t\/opt|^\t\/usr\/local/{print $1}'); do
    [ -e "$l" ] || echo "  $l      (macOS: brew install sdl2-compat)"
  done
  exit 1
fi
"$QEMU_DIR/qemu/bin/qemu-system-xtensa" --version | head -1

cat <<TXT

Use it with:
  export PATH="\$HOME/.pio-venv/bin:\$PATH"
  export QEMU="\$HOME/.qemu-esp/qemu/bin/qemu-system-xtensa"
  cd firmware/esp32 && pio run -e esp32s3            # the robot image
  hil/build_flash.sh                                 # the QEMU image (env esp32s3_sim)
  .venv/bin/python hil/run_hil.py                    # the plant needs the project venv's pinned mujoco
TXT
