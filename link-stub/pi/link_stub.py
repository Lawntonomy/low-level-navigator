#!/usr/bin/env python3
"""
Lawntonomy inter-tier link stub - Raspberry Pi side.

Drives the RP2350 link stub so the decisions in IF-0001 (protocol) and TP-0001
(transport) can be measured on real hardware. It is not the planner: the drive
requests it sends are a slow sine, and nothing here estimates anything.

What it exercises:
  IF-0001 §3    identity (sysid 1, compid 191)
  IF-0001 §7.1  heartbeat at T_hb, independent of command traffic
  IF-0001 §7.4  four-timestamp TIMESYNC, offset AND skew by least squares
  IF-0001 §7.5  session id in HEARTBEAT.custom_mode; stall vs restart
  TP-0001 T0.5  termios that will not corrupt a binary stream

Usage:
    ./link_stub.py --port /dev/rp2350-cmd
    ./link_stub.py --port /dev/ttyAMA1 --stall 3.0   # inject a 3 s freeze
"""

import argparse
import math
import os
import random
import sys
import time
from collections import deque

try:
    import serial  # pyserial
except ImportError:
    sys.exit("pyserial missing:  pip install pyserial")

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "..", "mavlink", "python"))
try:
    from lawntonomy import MAVLink, MAVLINK_MSG_ID_LAWN_TIMESYNC  # noqa: F401
    import lawntonomy as dialect
except ImportError:
    sys.exit("dialect missing - run ../../generate-mavlink.sh first")

SYSID = 1
COMPID_PI = 191    # MAV_COMP_ID_ONBOARD_COMPUTER
COMPID_PICO = 1    # MAV_COMP_ID_AUTOPILOT1

T_HB = 0.050       # IF-0001 §7.1, PROVISIONAL pending SAF-2
T_CMD = 0.050
T_SYNC = 0.200     # 5 Hz


def now_us() -> int:
    """Monotonic microseconds. Never wall clock: it steps."""
    return time.monotonic_ns() // 1000


class ClockModel:
    """
    Fits offset AND skew between the Pi's clock and the RP2350's.

    IF-0001 §7.4: a single exchange cannot beat ~1 ms, so the estimator must
    never consume a raw per-exchange offset. Samples are min-filtered on
    round-trip delay (NTP style) because the lowest-delay samples are the ones
    least contaminated by queuing, then fitted by least squares.
    """

    def __init__(self, window=200, keep_best=0.25):
        self.samples = deque(maxlen=window)
        self.keep_best = keep_best
        self.offset_us = None
        self.skew_ppm = None
        self.last_delay_us = None

    def add(self, t1, t2, t3, t4):
        offset = ((t2 - t1) + (t3 - t4)) / 2.0
        delay = (t4 - t1) - (t3 - t2)
        if delay < 0:
            return  # impossible; a clock moved or a frame was misattributed
        self.last_delay_us = delay
        self.samples.append((t1, offset, delay))
        self._fit()

    def _fit(self):
        if len(self.samples) < 8:
            return
        best = sorted(self.samples, key=lambda s: s[2])
        best = best[: max(4, int(len(best) * self.keep_best))]

        n = len(best)
        t0 = best[0][0]
        xs = [(s[0] - t0) for s in best]
        ys = [s[1] for s in best]
        mx = sum(xs) / n
        my = sum(ys) / n
        den = sum((x - mx) ** 2 for x in xs)
        if den == 0:
            return
        slope = sum((x - mx) * (y - my) for x, y in zip(xs, ys)) / den
        self.offset_us = my + slope * (now_us() - t0 - mx)
        self.skew_ppm = slope * 1e6

    def summary(self):
        if self.offset_us is None:
            return f"converging ({len(self.samples)} samples)"
        return (f"offset {self.offset_us / 1000:+.3f} ms  "
                f"skew {self.skew_ppm:+.2f} ppm  "
                f"delay {self.last_delay_us:.0f} us  "
                f"n={len(self.samples)}")


def open_port(path, baud):
    """
    TP-0001 T0.5: cfmakeraw() is necessary and not sufficient. pyserial gives a
    raw 8N1 port, but these three have to be off explicitly or a binary wire
    format corrupts in ways that look like a firmware bug:

      rtscts  - CTS gating on an unmuxed pin stalls TX permanently, silently
      xonxoff - a payload byte of 0x13 (XOFF) halts transmission until 0x11
      dsrdtr  - the Pi's PL011 implements no modem lines
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
    ser.timeout = 0        # non-blocking; we poll
    ser.write_timeout = 0.05
    try:
        ser.open()
    except serial.SerialException as e:
        sys.exit(f"cannot open {path}: {e}\n"
                 f"  - is 'dtoverlay=uart2' in /boot/firmware/config.txt? (TP-0001 T0.1)\n"
                 f"  - check the node with: dmesg | grep ttyAMA\n"
                 f"  - prefer a stable udev name over ttyAMA* (TP-0001 T0.3)")

    got = ser.baudrate
    if abs(got - baud) > baud * 0.02:
        sys.exit(f"baud clamped: asked {baud}, got {got} "
                 f"(PL011 ceiling is uartclk/16 = 3000000)")
    print(f"[open] {path} at {got} baud, no flow control")
    return ser


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", default="/dev/rp2350-cmd")
    ap.add_argument("--baud", type=int, default=1000000)
    ap.add_argument("--no-arm", action="store_true",
                    help="never send LAWN_ARM_CMD; the Pico should refuse to move")
    ap.add_argument("--stall", type=float, default=0.0, metavar="SEC",
                    help="freeze this process mid-run to test §7.2 rule 5 "
                         "(drain-to-newest) and §7.5 stop-vs-disarm")
    ap.add_argument("--stall-at", type=float, default=10.0, metavar="SEC")
    args = ap.parse_args()

    ser = open_port(args.port, args.baud)
    mav = MAVLink(ser, srcSystem=SYSID, srcComponent=COMPID_PI)
    mav.robust_parsing = True

    session = random.getrandbits(32)
    print(f"[open] our session {session:08x}")

    clock = ClockModel()
    pending = {}          # exchange_seq -> t1
    sync_seq = 0

    peer_session = None
    rx_counts = {}
    last_stats = None
    frames_rx = 0
    bad_rx = 0

    t_start = time.monotonic()
    next_hb = next_cmd = next_sync = next_print = time.monotonic()
    armed_sent = False
    stalled = False

    try:
        while True:
            nowm = time.monotonic()

            # ---- injected stall -------------------------------------------
            if args.stall and not stalled and (nowm - t_start) > args.stall_at:
                stalled = True
                print(f"\n[test] freezing {args.stall:.1f}s - expect the Pico to "
                      f"ramp to zero and (past T_disarm) disarm\n")
                time.sleep(args.stall)
                # Everything queued during the freeze now drains as a burst.
                # IF-0001 §7.2 rule 5 says the Pico must collapse it to the
                # newest request rather than replaying stale steering.
                continue

            # ---- transmit --------------------------------------------------
            if nowm >= next_hb:
                mav.heartbeat_send(
                    type=dialect.MAV_TYPE_ONBOARD_CONTROLLER,
                    autopilot=dialect.MAV_AUTOPILOT_INVALID,
                    base_mode=0,
                    custom_mode=session,        # §7.5 session id
                    system_status=dialect.MAV_STATE_ACTIVE)
                next_hb = nowm + T_HB

            if not armed_sent and not args.no_arm and (nowm - t_start) > 1.0:
                mav.lawn_arm_cmd_send(magic=0xA57E, arm=1)
                armed_sent = True
                print("[test] sent LAWN_ARM_CMD")

            if nowm >= next_cmd:
                phase = (nowm - t_start) * 0.4
                l = int(1200 * math.sin(phase))
                r = int(1200 * math.sin(phase + 0.6))
                mav.lawn_drive_cmd_send(left_drpm=l, right_drpm=r)
                next_cmd = nowm + T_CMD

            if nowm >= next_sync:
                sync_seq = (sync_seq + 1) & 0xFF
                t1 = now_us()
                pending[sync_seq] = t1
                mav.lawn_timesync_send(t1_us=t1, t2_us=0, t3_us=0,
                                       exchange_seq=sync_seq)
                next_sync = nowm + T_SYNC

            # ---- receive ---------------------------------------------------
            data = ser.read(4096)
            if data:
                t4 = now_us()
                try:
                    msgs = mav.parse_buffer(data) or []
                except Exception:
                    msgs = []
                for m in msgs:
                    name = m.get_type()
                    if name == "BAD_DATA":
                        bad_rx += 1
                        continue
                    if m.get_srcSystem() != SYSID or m.get_srcComponent() != COMPID_PICO:
                        bad_rx += 1
                        continue

                    frames_rx += 1
                    rx_counts[name] = rx_counts.get(name, 0) + 1

                    if name == "HEARTBEAT":
                        if peer_session is None:
                            peer_session = m.custom_mode
                            print(f"[link] Pico session {peer_session:08x}")
                        elif m.custom_mode != peer_session:
                            print(f"\n[link] Pico RESTARTED "
                                  f"{peer_session:08x} -> {m.custom_mode:08x}"
                                  f" - re-arming\n")
                            peer_session = m.custom_mode
                            clock = ClockModel()   # its clock restarted too
                            armed_sent = False

                    elif name == "LAWN_TIMESYNC":
                        t1 = pending.pop(m.exchange_seq, None)
                        if t1 is not None and m.t2_us and m.t3_us:
                            clock.add(t1, m.t2_us, m.t3_us, t4)

                    elif name == "LAWN_LINK_STATS":
                        last_stats = m

            # ---- report ----------------------------------------------------
            if nowm >= next_print:
                el = nowm - t_start
                line = (f"[{el:6.1f}s] rx={frames_rx:<6} bad={bad_rx:<4} "
                        f"| {clock.summary()}")
                if last_stats:
                    line += (f" | pico q={last_stats.quality}% "
                             f"ok={last_stats.frames_accepted} "
                             f"bad={last_stats.frames_rejected} "
                             f"txdrop={last_stats.dropped_tx}")
                print(line)
                next_print = nowm + 1.0

            time.sleep(0.001)

    except KeyboardInterrupt:
        print("\n[done]")
        for k in sorted(rx_counts):
            print(f"  {k:<22} {rx_counts[k]}")
        if bad_rx:
            print(f"  {'BAD/rejected':<22} {bad_rx}")
    finally:
        ser.close()


if __name__ == "__main__":
    main()
