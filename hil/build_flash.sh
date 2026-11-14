#!/bin/bash
# Build the HIL firmware and merge it into an 8 MB QEMU flash image (/tmp/flash_sim.bin).
# ROBOT_MOTOR picks the env, the same variable that picks the plant, so the two cannot drift apart by
# accident: run_hil.py reads the motor back out of the image's boot line and refuses a mismatch.
set -e
MOTOR=${ROBOT_MOTOR:-mf6015}
case "$MOTOR" in
  mf6015) ENV=esp32s3_sim ;;
  mf7015) ENV=esp32s3_sim_mf7015 ;;
  *) echo "ROBOT_MOTOR=$MOTOR: no HIL env for that motor (see firmware/esp32/platformio.ini)"; exit 1 ;;
esac
cd "$(dirname "$0")/../firmware/esp32"
grep -q "MOTOR = \"$MOTOR\"" lib/botcore/gains.hpp || {
  echo "lib/botcore/gains.hpp was designed on another plant, not $MOTOR's. Regenerate and sync:"
  echo "  cd sim && ROBOT_MOTOR=$MOTOR python build_model.py && ROBOT_MOTOR=$MOTOR python design_lqr.py 0.06 0.6 10"
  echo "  firmware/esp32/sync_core.sh"
  exit 1
}
pio run -e $ENV
B=.pio/build/$ENV
BA=$(find ~/.platformio/packages/framework-arduinoespressif32 -name boot_app0.bin | head -1)
python3 ~/.platformio/packages/tool-esptoolpy/esptool.py --chip esp32s3 merge_bin --fill-flash-size 8MB -o /tmp/flash_sim.bin \
  --flash_mode keep --flash_freq keep --flash_size keep 0x0 $B/bootloader.bin 0x8000 $B/partitions.bin 0xe000 "$BA" 0x10000 $B/firmware.bin
echo "flash image: /tmp/flash_sim.bin   run: python3 ../../hil/run_hil.py"
