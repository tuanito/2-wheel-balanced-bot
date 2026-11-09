"""Open a CAN adapter with python-can from a short spec string.

  socketcan:can0          Linux SocketCAN (candleLight/CANable with gs_usb firmware, PEAK, Kvaser...): first
                          `sudo ip link set can0 up type can bitrate 1000000`
  gs_usb:0                candleLight/CANable (gs_usb firmware) directly over libusb (macOS, Windows): pip install gs_usb
  slcan:/dev/ttyACM0      CANable with slcan firmware (any OS), e.g. slcan:COM5 on Windows
  pcan:PCAN_USBBUS1       PEAK PCAN-USB
  virtual:test            python-can's in-process virtual bus (tests)
"""
import can

BITRATE = 1_000_000


def open_bus(spec):
    kind, _, chan = spec.partition(":")
    kw = dict(interface=kind, channel=chan or None)
    if kind == "gs_usb":
        kw["channel"] = int(chan or 0); kw["index"] = int(chan or 0); kw["bitrate"] = BITRATE
    elif kind in ("slcan", "pcan", "kvaser", "ixxat", "vector"):
        kw["bitrate"] = BITRATE
    elif kind == "virtual":
        kw["receive_own_messages"] = False
    return can.Bus(**kw)
