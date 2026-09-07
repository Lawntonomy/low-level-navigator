#!/usr/bin/env python3
"""
Stream and decode LAWN_WHEEL_STATE from the command link, with receipt
timestamps -- the observation tool for TP-0002 CAL-0's acceptance criterion:
a stopped wheel reads zero and clears its VALID flag within t_stale.

This only DECODES and prints; it makes no pass/fail call. t_stale itself is
still an open question in TP-0002 ("needs a derivation from the slowest speed
the control loop must resolve, not a round number") -- the value currently
compiled into the firmware is src/hardware_drivers/encoder_math.hpp's
t_stale_us (1_000_000 as of this writing), which --t-stale-ms defaults to
purely as a display aid; re-check that header if the printed elapsed times
look like they are being judged against the wrong number.

Must run ON THE PI: it opens a local serial device (the command link),
which does not exist anywhere else. It is not meant to be invoked directly --
scripts/bench/cal0-check.sh copies it over (alongside mavlink/python/, three
directories up from here, which is where the sys.path.insert below expects
it) and runs it there.

Usage (on the Pi, after scripts/bench/cal0-check.sh has copied this over):
    python3 -u wheel_state_monitor.py --port /dev/ttyAMA2 --seconds 30
"""

import argparse
import datetime
import os
import sys
import time

try:
    import serial  # pyserial
except ImportError:
    sys.exit("pyserial missing: pip install pyserial")

# Mirrors the ../mavlink/python layout used elsewhere in this repo (see
# scripts/enter-bootloader.py) -- but this file lives three directories down
# from the repo root (scripts/bench/pi/), not one, hence the extra "..".
sys.path.insert(
    0,
    os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "..", "..", "mavlink", "python"),
)
try:
    from lawntonomy import MAVLink
    import lawntonomy as dialect
except ImportError:
    sys.exit("dialect missing - run ../generate-mavlink.sh first, on whichever side has it checked out")

SYSID = 1
COMPID_PICO = 1  # MAV_COMP_ID_AUTOPILOT1


def open_port(path: str, baud: int) -> "serial.Serial":
    # Same reasoning as scripts/enter-bootloader.py's open_port: cfmakeraw()
    # equivalent settings, no flow control (the Pi's PL011 implements no
    # modem lines and rtscts stalls TX on an unmuxed pin).
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


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--port", default="/dev/ttyAMA2")
    ap.add_argument("--baud", type=int, default=1_000_000)
    ap.add_argument("--seconds", type=float, default=0.0, help="0 = run until Ctrl-C")
    ap.add_argument(
        "--t-stale-ms",
        type=float,
        default=1000.0,
        help="display aid only -- see the header comment; not authoritative",
    )
    args = ap.parse_args()

    ser = open_port(args.port, args.baud)
    parser = MAVLink(None)

    print(f"[wheel-state-monitor] listening on {args.port} @ {args.baud}, t_stale_ms={args.t_stale_ms} (display only)")
    print("recv_time_utc,t_meas_us,left_drpm,right_drpm,left_valid,right_valid,left_stall,right_stall,left_ms_since_valid,right_ms_since_valid")

    last_valid_at = {"left": None, "right": None}
    deadline = (time.monotonic() + args.seconds) if args.seconds > 0 else None

    try:
        while deadline is None or time.monotonic() < deadline:
            data = ser.read(256)
            if not data:
                time.sleep(0.005)
                continue
            try:
                msgs = parser.parse_buffer(data) or []
            except Exception:
                # A parse error is an expected transient (partial frame at
                # the start of the stream), not a fault worth a traceback.
                continue

            recv_time = time.monotonic()
            for msg in msgs:
                if msg.get_type() != "LAWN_WHEEL_STATE":
                    continue
                if msg.get_srcSystem() != SYSID or msg.get_srcComponent() != COMPID_PICO:
                    continue

                left_valid = bool(msg.flags & dialect.LAWN_WHEEL_LEFT_VALID)
                right_valid = bool(msg.flags & dialect.LAWN_WHEEL_RIGHT_VALID)
                left_stall = bool(msg.flags & dialect.LAWN_WHEEL_LEFT_STALL)
                right_stall = bool(msg.flags & dialect.LAWN_WHEEL_RIGHT_STALL)

                if left_valid:
                    last_valid_at["left"] = recv_time
                if right_valid:
                    last_valid_at["right"] = recv_time

                def ms_since(key: str) -> str:
                    t0 = last_valid_at[key]
                    return f"{(recv_time - t0) * 1000.0:.0f}" if t0 is not None else ""

                now_iso = datetime.datetime.now(datetime.timezone.utc).isoformat(timespec="milliseconds")
                print(
                    f"{now_iso},{msg.t_meas_us},{msg.left_drpm},{msg.right_drpm},"
                    f"{int(left_valid)},{int(right_valid)},{int(left_stall)},{int(right_stall)},"
                    f"{ms_since('left')},{ms_since('right')}"
                )
                sys.stdout.flush()
    except KeyboardInterrupt:
        print("[wheel-state-monitor] stopped")

    return 0


if __name__ == "__main__":
    sys.exit(main())
