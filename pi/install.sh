#!/bin/bash
# One-time setup on a fresh Raspberry Pi OS Bookworm Lite (64-bit). Run from /home/pi/robot/pi
set -e
sudo apt update
sudo apt install -y python3-picamera2 python3-numpy python3-pil python3-serial python3-aiohttp python3-venv git
# UART to the ESP32 on GPIO14/15, no login shell on it; SPI for the LED eyes
sudo raspi-config nonint do_serial_hw 0
sudo raspi-config nonint do_serial_cons 1
sudo raspi-config nonint do_spi 0
grep -q "^dtoverlay=miniuart-bt" /boot/firmware/config.txt || printf "dtoverlay=miniuart-bt\ncore_freq=250\n" | sudo tee -a /boot/firmware/config.txt   # full PL011 UART for the ESP32 on GPIO14/15, Bluetooth (gamepad) moves to the mini UART
# I2S microphones (2 x ICS-43434) + MAX98357A speaker amp: same pair as the Google AIY Voice Kit
grep -q "^dtoverlay=googlevoicehat-soundcard" /boot/firmware/config.txt || echo "dtoverlay=googlevoicehat-soundcard" | sudo tee -a /boot/firmware/config.txt
sudo apt install -y python3-scipy libportaudio2 espeak-ng unzip
# AI Camera (IMX500): firmware + model zoo (person/cat detector), OpenCV for face ID
sudo apt install -y imx500-all python3-opencv
# headlights: hardware PWM0 on GPIO12 (pin 32) -> IRLB8721 gate
grep -q "^dtoverlay=pwm,pin=12" /boot/firmware/config.txt || echo "dtoverlay=pwm,pin=12,func=4" | sudo tee -a /boot/firmware/config.txt
# offline speech recognition model (English; for German use vosk-model-small-de-0.15 and set "language": "de")
mkdir -p /home/pi/models && cd /home/pi/models
[ -d vosk-model-small-en-us-0.15 ] || (wget -q https://alphacephei.com/vosk/models/vosk-model-small-en-us-0.15.zip && unzip -q vosk-model-small-en-us-0.15.zip && rm vosk-model-small-en-us-0.15.zip)
# neural voice (piper): binary + a low-quality fast voice
[ -x /usr/local/bin/piper ] || (wget -q https://github.com/rhasspy/piper/releases/latest/download/piper_linux_aarch64.tar.gz && tar xzf piper_linux_aarch64.tar.gz && sudo cp -r piper /opt/ && sudo ln -sf /opt/piper/piper /usr/local/bin/piper && rm -rf piper piper_linux_aarch64.tar.gz)
[ -f en_US-lessac-low.onnx ] || (wget -q https://huggingface.co/rhasspy/piper-voices/resolve/main/en/en_US/lessac/low/en_US-lessac-low.onnx && wget -q https://huggingface.co/rhasspy/piper-voices/resolve/main/en/en_US/lessac/low/en_US-lessac-low.onnx.json)
# face detection + recognition models (OpenCV Zoo, ~40 MB)
[ -f face_detection_yunet_2023mar.onnx ] || wget -q https://github.com/opencv/opencv_zoo/raw/main/models/face_detection_yunet/face_detection_yunet_2023mar.onnx
[ -f face_recognition_sface_2021dec.onnx ] || wget -q https://github.com/opencv/opencv_zoo/raw/main/models/face_recognition_sface/face_recognition_sface_2021dec.onnx
cd - >/dev/null
# Claude API key: kept out of config.json, readable by root only, loaded by robot.service
[ -f /etc/robot.env ] || (echo "ANTHROPIC_API_KEY=" | sudo tee /etc/robot.env >/dev/null && sudo chmod 600 /etc/robot.env && echo ">>> put your API key in /etc/robot.env")
sudo usermod -aG dialout,video,spi,gpio,audio pi
python3 -m venv --system-site-packages ../venv
../venv/bin/pip install -r requirements.txt
sudo cp robot.service /etc/systemd/system/ && sudo systemctl daemon-reload && sudo systemctl enable robot
echo "Reboot, then open http://$(hostname).local:8080"
