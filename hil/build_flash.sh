#!/bin/bash
# Build the HIL firmware (env esp32s3_sim) and merge it into an 8 MB QEMU flash image (/tmp/flash_sim.bin).
set -e
cd "$(dirname "$0")/../firmware/esp32"
pio run -e esp32s3_sim
B=.pio/build/esp32s3_sim
BA=$(find ~/.platformio/packages/framework-arduinoespressif32 -name boot_app0.bin | head -1)
python3 ~/.platformio/packages/tool-esptoolpy/esptool.py --chip esp32s3 merge_bin --fill-flash-size 8MB -o /tmp/flash_sim.bin \
  --flash_mode keep --flash_freq keep --flash_size keep 0x0 $B/bootloader.bin 0x8000 $B/partitions.bin 0xe000 "$BA" 0x10000 $B/firmware.bin
echo "flash image: /tmp/flash_sim.bin   run: python3 ../../hil/run_hil.py"
