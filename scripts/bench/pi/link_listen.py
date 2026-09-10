#!/usr/bin/env python3
"""
Passive listener on the inter-tier command link. Sends NOTHING.

Answers, without touching the board: is the Pico talking, which messages is it
emitting and at what rate, is it armed, and has it restarted (session id).

Deliberately dependency-free -- pure stdlib, no pymavlink, no dialect module,
no venv on the Pi to keep in sync. That is worth a small loss of rigour: frames
are located by their length field and NOT checksum-validated, because CRC_EXTRA
is per-message and lives in the generated dialect this script exists to avoid
needing. A stray byte can therefore invent a frame. It shows up as a one-off
count against an implausible message id, which is easy to read past when the
real traffic is arriving at a steady 20-50 Hz.

Reading the port is not opening a serial console: nothing here writes, so the
machine cannot be armed or moved by running it.

Usage:
    python3 -u link_listen.py --port /dev/ttyAMA2 --seconds 5
"""

import argparse
import struct
import sys
import time

try:
    import serial  # pyserial
except ImportError:
    sys.exit("pyserial missing on this host:  pip install pyserial")

V1_STX = 0xFE
V2_STX = 0xFD

# IF-0001 §6. Only what a status report needs to name; anything else prints
# as its bare id rather than being dropped.
NAMES = {
    0: "HEARTBEAT",
    42010: "LAWN_NAV_STATUS",
    42011: "LAWN_WHEEL_STATE",
    42012: "LAWN_LINK_STATS",
    42013: "LAWN_FAULT_EVENT",
    42020: "LAWN_TIMESYNC",
}
EXPECTED_HZ = {0: 20, 42010: 20, 42011: 50, 42012: 1}

MAV_MODE_FLAG_SAFETY_ARMED = 0x80

# MAV_STATE. link.cpp reports MAV_STATE_CRITICAL whenever safety::Fault is not
# none, and MAV_STATE_ACTIVE otherwise, so this byte is the cheapest read of
# "is a fault latched" that needs no dialect.
MAV_STATE = {0: "UNINIT", 1: "BOOT", 2: "CALIBRATING", 3: "STANDBY", 4: "ACTIVE",
             5: "CRITICAL", 6: "EMERGENCY", 7: "POWEROFF", 8: "FLIGHT_TERMINATION"}


def openPort(path, baud):
    """
    TP-0001 T0.5: cfmakeraw is necessary and not sufficient. rtscts stalls TX on
    an unmuxed pin, a payload byte of 0x13 halts an xonxoff port, and the Pi's
    PL011 implements no modem lines. All three off, explicitly -- even here,
    where nothing is transmitted, because flow control also gates RX.
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
    try:
        ser.open()
    except serial.SerialException as e:
        sys.exit(f"cannot open {path}: {e}")
    return ser


def frames(buf):
    """Yield (sysid, compid, msgid, payload) and return the unconsumed tail."""
    i = 0
    out = []
    while i < len(buf):
        stx = buf[i]
        if stx == V2_STX:
            if len(buf) - i < 12:
                break
            ln = buf[i + 1]
            incompat = buf[i + 2]
            total = 12 + ln + (13 if incompat & 0x01 else 0)
            if len(buf) - i < total:
                break
            sysid, compid = buf[i + 5], buf[i + 6]
            msgid = buf[i + 7] | (buf[i + 8] << 8) | (buf[i + 9] << 16)
            out.append((sysid, compid, msgid, bytes(buf[i + 10:i + 10 + ln])))
            i += total
        elif stx == V1_STX:
            if len(buf) - i < 8:
                break
            ln = buf[i + 1]
            total = 8 + ln
            if len(buf) - i < total:
                break
            sysid, compid, msgid = buf[i + 3], buf[i + 4], buf[i + 5]
            out.append((sysid, compid, msgid, bytes(buf[i + 6:i + 6 + ln])))
            i += total
        else:
            i += 1  # resync
    return out, buf[i:]


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", default="/dev/ttyAMA2")
    ap.add_argument("--baud", type=int, default=1000000)
    ap.add_argument("--seconds", type=float, default=5.0)
    args = ap.parse_args()

    ser = openPort(args.port, args.baud)
    print(f"listening on {args.port} at {args.baud} for {args.seconds}s (transmitting nothing)")

    counts, senders = {}, set()
    session, armed, hb_seen, state = None, None, 0, None
    buf = bytearray()
    deadline = time.monotonic() + args.seconds
    while time.monotonic() < deadline:
        data = ser.read(4096)
        if not data:
            time.sleep(0.002)
            continue
        buf.extend(data)
        got, buf = frames(buf)
        buf = bytearray(buf)
        for sysid, compid, msgid, payload in got:
            counts[msgid] = counts.get(msgid, 0) + 1
            senders.add((sysid, compid))
            if msgid == 0:
                hb_seen += 1
                # HEARTBEAT wire order after MAVLink's size-descending field
                # sort: custom_mode u32 (0..3), type (4), autopilot (5),
                # base_mode (6), system_status (7), mavlink_version (8). v2
                # truncates trailing zeros, so a short payload means those
                # fields were 0 -- which for base_mode reads as disarmed, and
                # is the correct reading.
                if len(payload) >= 4:
                    session = struct.unpack_from("<I", payload, 0)[0]
                base = payload[6] if len(payload) >= 7 else 0
                armed = bool(base & MAV_MODE_FLAG_SAFETY_ARMED)
                state = payload[7] if len(payload) >= 8 else 0

    total = sum(counts.values())
    print()
    if total == 0:
        print("NOTHING RECEIVED.")
        print("  Either the Pico is not running, the resident firmware does not emit")
        print("  telemetry, or the port/baud is wrong. This does NOT by itself mean the")
        print("  board is dead -- a BOOTSEL-resident or stdio-only image is silent here.")
        return 1

    print(f"senders   {', '.join(f'sysid {s} compid {c}' for s, c in sorted(senders))}")
    print(f"frames    {total} in {args.seconds:.1f}s")
    print()
    print(f"  {'message':<20} {'count':>6} {'Hz':>7}   expected")
    for msgid in sorted(counts):
        hz = counts[msgid] / args.seconds
        exp = EXPECTED_HZ.get(msgid)
        print(f"  {NAMES.get(msgid, str(msgid)):<20} {counts[msgid]:>6} {hz:>7.1f}   "
              f"{exp if exp else '-'}")

    print()
    if hb_seen:
        print(f"session id    0x{session:08X}  (changes only when the Pico restarts)")
        print(f"armed         {'YES -- disarm before doing anything else' if armed else 'no'}")
        print(f"system_status {state} ({MAV_STATE.get(state, '?')})")
        if state == 5:
            print("              CRITICAL means a fault is LATCHED. In protocol v1 a latched")
            print("              fault needs a deliberate reset; the machine will not arm.")
            print("              Read LAWN_NAV_STATUS.fault for the code (7 = init failed).")
    else:
        print("no HEARTBEAT seen: arm state and session id unknown")
    return 0


if __name__ == "__main__":
    sys.exit(main())
