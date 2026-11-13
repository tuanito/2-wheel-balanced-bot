#!/bin/sh
# Downloads the bought-part 3D models used for the fit check (models/bought/) and the official drawings (models/drawings/).
set -e
cd "$(dirname "$0")"; mkdir -p bought drawings tmp
# Raspberry Pi Camera Module 3 (official STEP, Raspberry Pi product information portal)
curl -sL -o tmp/cam3.zip "https://pip-assets.raspberrypi.com/categories/1207-design-files/documents/RP-008154-DS-1-camera-module-3-step.zip"
unzip -o -q tmp/cam3.zip -d tmp/cam3 && cp tmp/cam3/Camera_module_3_wide_model_simple.stp bought/rpi_camera_module_3_wide.step
# Raspberry Pi Zero 2 W (no official STEP: official mechanical drawing)
curl -sL -o drawings/rpi_zero2w_mech.pdf "https://pip-assets.raspberrypi.com/categories/584-raspberry-pi-zero-2-w/documents/RP-008358-DS-1-raspberry-pi-zero-2-w-mechanical-drawing.pdf"
# ESP32-S3-DevKitC-1 (no official STEP: Espressif dimension drawing) + ESP32-S3-WROOM-1 module (Espressif KiCad library)
curl -sL -o drawings/esp32s3_devkitc1.dxf "https://dl.espressif.com/dl/schematics/esp_idf/DXF_ESP32-S3-DevKitC-1_V1.1_20220429.dxf"
curl -sL -o drawings/esp32s3_devkitc1.pdf "https://dl.espressif.com/dl/schematics/esp_idf/DXF_ESP32-S3-DevKitC-1_V1.1_20220429.pdf"
curl -sL -o bought/espressif_esp32s3_wroom1.step "https://raw.githubusercontent.com/espressif/kicad-libraries/main/3dmodels/espressif.3dshapes/ESP32-S3-WROOM-1.STEP"
# LD19 lidar and SN65HVD230 CAN board (Waveshare wiki)
curl -sL -o tmp/ld19.zip "https://files.waveshare.com/upload/d/d2/DTOF-LIDAR-LD19-STEP.zip" && unzip -o -q tmp/ld19.zip -d tmp/ld19 && cp tmp/ld19/LD19.step bought/ldrobot_ld19_waveshare.step
curl -sL -o bought/waveshare_sn65hvd230_can_board.step "https://files.waveshare.com/upload/c/ca/CAN%20Transiver%20SN65HVD230.step"
# Pololu D24V50F5 (Pololu)
curl -sL -A "Mozilla/5.0" -o bought/pololu_d24v50f5.step "https://www.pololu.com/file/0J1437/d24v50f5-step-down-voltage-regulator.step"
# Adafruit 3006 MAX98357A, 3968 speaker, 6049 ICS-43434 (Adafruit_CAD_Parts on GitHub)
A="https://raw.githubusercontent.com/adafruit/Adafruit_CAD_Parts/main"
curl -sL -o bought/adafruit_3006_max98357.step "$A/3006%20MAX98357/3006%20MAX98357.step"
curl -sL -o bought/adafruit_3968_speaker40.step "$A/3968%20Speaker%2040mm/3968%20Speaker%2040mm.step"
curl -sL -o bought/adafruit_6049_ics43434.step "$A/6049%20I2S%20MEMS%20Microphone/6049%20I2S%20MEMS%20Microphone.step"
# HC-SR04 (FreeCAD-library, community model)
curl -sL -o bought/hc_sr04_freecadlib.step "https://raw.githubusercontent.com/FreeCAD/FreeCAD-library/master/Sensors/Ultrasonic%20Sensors/HC-SR04.step"
# MF7015-V2 / RMD-L-7015 motor: official STEP + 2D drawing + parameter sheet, straight from the vendor.
# Public download, unlike the MF6015's, so it can live in the script. The archive has GBK filenames
# that unzip(1) refuses, hence the python extraction.
curl -sL -A "Mozilla/5.0" -o tmp/l7015.zip "https://www.myactuator.com/_files/archives/cab28a_da333fb383e74af4ac6e9e89d8dcba32.zip?dn=L-7015-251029.zip"
python3 -c "
import re, sys, zipfile, pathlib
z = zipfile.ZipFile('tmp/l7015.zip')
for i in z.infolist():
    raw = i.filename.encode() if i.flag_bits & 0x800 else i.filename.encode('cp437', 'replace')
    try: name = raw.decode('gbk')
    except Exception: name = raw.decode('utf-8', 'replace')
    if name.endswith('RMD-7015.step'): pathlib.Path('bought/mf7015.step').write_bytes(z.read(i))
    elif name.endswith('RMD-7015.pdf'): pathlib.Path('drawings/myactuator_l7015_dimensions.pdf').write_bytes(z.read(i))
    elif name.endswith('.pdf') and 'parameter' in name.lower(): pathlib.Path('drawings/myactuator_l7015_parameters.pdf').write_bytes(z.read(i))
"
# MF6015 motor: STEP from LKMTECH / the dealer's product page (already used for the CAD).
rm -rf tmp; echo done
