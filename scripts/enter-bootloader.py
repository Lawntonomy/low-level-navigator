#!/usr/bin/env python3
"""
Ask the low-level navigator to reboot into BOOTSEL, over the command link.

The main firmware does not enable USB stdio (TP-0001 D5), so `picotool reboot`
has nothing to talk to and reflashing otherwise needs a finger on the button.
This sends LAWN_ENTER_BOOTLOADER; the RP2350 bootrom then brings USB up itself
after the reset, with mass storage suppressed so no spurious drive appears.

There is deliberately no acknowledgement to wait for. Acceptance is signalled
by the PICOBOOT device appearing (0x2e8a:0x000f, TP-0001 T5.3), and an ack
would race a reset that does not return. What this script does wait for is a
REFUSAL, which the firmware reports as a non-latching LAWN_FAULT_EVENT with
code LAWN_FAULT_BOOTLOADER_REFUSED. Refusals happen when the magic is wrong or
the machine is armed; disarm first.

Usage:
    ./scripts/enter-bootloader.py --port /dev/rp2350-cmd
    ./scripts/enter-bootloader.py --port /dev/ttyAMA1 --wait 1.0

Then, once the device enumerates:
    picotool load -f build/low-level-nav.uf2
"""

import argparse
import os
import sys
import time

try:
    import serial  # pyserial
except ImportError:
    sys.exit("pyserial missing:  pip install pyserial")

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "mavlink", "python"))
try:
    from lawntonomy import MAVLink
    import lawntonomy as dialect
except ImportError:
    sys.exit("dialect missing - run ../generate-mavlink.sh first")

SYSID = 1
COMPID_PI = 191     # MAV_COMP_ID_ONBOARD_COMPUTER
COMPID_PICO = 1     # MAV_COMP_ID_AUTOPILOT1

# Must match bootloader::request_magic in src/app/bootloader.hpp. Deliberately
# unrelated to LAWN_ARM_CMD.magic so neither can be mistaken for the other.
REQUEST_MAGIC = 0xB00710AD


def open_port(path, baud):
    """
    TP-0001 T0.5: cfmakeraw() is necessary and not sufficient. rtscts stalls TX
    on an unmuxed pin, a payload byte of 0x13 halts an xonxoff port, and the
    Pi's PL011 implements no modem lines. All three off, explicitly.
    """
    ser = serial.Serial()
    ser.port = path
    ser.baudrate = baud
    ser.bytesize = serial.EIGHTBITS
    ser.parity = serial.PARITY_NONE
    ser.stopbits = serial.STOPBITS_ONE
    ser.rtscts = False
    ser.xonxoff = False
    ser.dsrdtr = False
    ser.timeout = 0
    ser.write_timeout = 0.5
    try:
        ser.open()
    except serial.SerialException as e:
        sys.exit(f"cannot open {path}: {e}")
    return ser


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--port", default="/dev/rp2350-cmd")
    ap.add_argument("--baud", type=int, default=1000000)
    ap.add_argument("--wait", type=float, default=0.5,
                    help="seconds to listen for a refusal before assuming acceptance")
    args = ap.parse_args()

    ser = open_port(args.port, args.baud)
    mav = MAVLink(None, srcSystem=SYSID, srcComponent=COMPID_PI)

    frame = mav.lawn_enter_bootloader_encode(REQUEST_MAGIC).pack(mav)
    ser.write(frame)
    ser.flush()
    print(f"[tx] LAWN_ENTER_BOOTLOADER magic=0x{REQUEST_MAGIC:08X} ({len(frame)} bytes)")

    # A refusal arrives promptly or not at all: the firmware answers from the
    # RX path and the TX task drains every millisecond.
    parser = MAVLink(None)
    deadline = time.monotonic() + args.wait
    while time.monotonic() < deadline:
        data = ser.read(256)
        if not data:
            time.sleep(0.005)
            continue
        for msg in parser.parse_buffer(data) or []:
            if msg.get_type() != "LAWN_FAULT_EVENT":
                continue
            if msg.code != dialect.LAWN_FAULT_BOOTLOADER_REFUSED:
                continue
            if msg.get_srcSystem() != SYSID or msg.get_srcComponent() != COMPID_PICO:
                continue
            print(f"[rx] REFUSED (nav_state={msg.nav_state}, latched={msg.latched})")
            print("     Disarm the machine and try again; check the magic if it stays disarmed.")
            return 2

    # Silence here is the expected outcome, not a failure: the controller has
    # reset and its UART is gone. Confirm with `picotool info` or lsusb.
    print("[ok] no refusal seen. Expect 2e8a:000f to enumerate; then:")
    print("     picotool load -f build/low-level-nav.uf2")
    return 0


if __name__ == "__main__":
    sys.exit(main())
