#!/usr/bin/env python3
"""
Dialect and budget self-test. No hardware required.

Validates the things IF-0001 asserts but cannot check about itself:
  - every message round-trips through encode/decode with fields intact
  - on-wire frame sizes match the table in IF-0001 §6
  - the §6 bandwidth budget arithmetic
  - a corrupted frame is rejected whole (SAF-53), not partially applied
  - frames from an unexpected component are rejected (IF-0001 §3)
  - the §4 maximum-frame claim (76 bytes / 760 us at 1 Mbaud)

Run:  ./selftest.py
"""

import io
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "..", "mavlink", "python"))
try:
    import lawntonomy as d
    from lawntonomy import MAVLink
except ImportError:
    sys.exit("dialect missing - run ../../generate-mavlink.sh first")

SYSID, COMPID_PICO, COMPID_PI = 1, 1, 191
OVERHEAD = 12          # MAVLink v2: 10 byte header + 2 byte CRC
WIRE_BPS = 100_000     # 1 Mbaud, 8N1 -> 10 bits per byte

fails = []


def check(cond, what):
    if cond:
        print(f"  ok   {what}")
    else:
        print(f"  FAIL {what}")
        fails.append(what)


def encoder(comp):
    buf = io.BytesIO()
    return MAVLink(buf, srcSystem=SYSID, srcComponent=comp), buf


def roundtrip(send, comp=COMPID_PICO):
    """Encode one message, return (decoded_message, wire_bytes)."""
    mav, buf = encoder(comp)
    send(mav)
    raw = buf.getvalue()
    rx = MAVLink(io.BytesIO(), srcSystem=SYSID, srcComponent=COMPID_PI)
    rx.robust_parsing = True
    msgs = [m for m in (rx.parse_buffer(raw) or []) if m.get_type() != "BAD_DATA"]
    assert len(msgs) == 1, f"expected 1 message, got {[m.get_type() for m in msgs]}"
    return msgs[0], raw


print("== round-trip: every field survives encode/decode ==")

m, _ = roundtrip(lambda v: v.lawn_drive_cmd_send(left_drpm=-1234, right_drpm=987),
                 COMPID_PI)
check(m.left_drpm == -1234 and m.right_drpm == 987,
      "LAWN_DRIVE_CMD signed deci-rpm round-trips (negative included)")

m, _ = roundtrip(lambda v: v.lawn_arm_cmd_send(magic=0xA57E, arm=1), COMPID_PI)
check(m.magic == 0xA57E and m.arm == 1, "LAWN_ARM_CMD magic survives")

BIG = 0x0123456789ABCDEF        # > 2^32: proves the field is really 64-bit
m, _ = roundtrip(lambda v: v.lawn_nav_status_send(
    t_meas_us=BIG, cmd_age_ms=4321, nav_state=d.LAWN_NAV_ACTIVE,
    armed=1, fault=d.LAWN_FAULT_NONE))
check(m.t_meas_us == BIG, "LAWN_NAV_STATUS 64-bit timestamp survives (D4)")
check(m.cmd_age_ms == 4321 and m.nav_state == d.LAWN_NAV_ACTIVE,
      "LAWN_NAV_STATUS remaining fields survive")

m, _ = roundtrip(lambda v: v.lawn_wheel_state_send(
    t_meas_us=BIG, left_drpm=-500, right_drpm=500,
    left_cmd_drpm=-600, right_cmd_drpm=600,
    flags=d.LAWN_WHEEL_LEFT_VALID | d.LAWN_WHEEL_DIR_UNMEASURED))
check(m.left_drpm == -500 and m.right_cmd_drpm == 600,
      "LAWN_WHEEL_STATE keeps requested and applied separate")
check(bool(m.flags & d.LAWN_WHEEL_DIR_UNMEASURED),
      "LAWN_WHEEL_STATE DIR_UNMEASURED bit survives (ADR-0002)")

m, _ = roundtrip(lambda v: v.lawn_link_stats_send(
    t_meas_us=BIG, frames_accepted=4_000_000_000, frames_rejected=7,
    dropped_tx=9, window_ms=1000, heartbeats_missed=3, quality=87))
check(m.frames_accepted == 4_000_000_000,
      "LAWN_LINK_STATS counter is genuinely 32-bit unsigned")
check(m.quality == 87 and m.dropped_tx == 9, "LAWN_LINK_STATS fields survive")

T1, T2, T3 = BIG, BIG + 1234, BIG + 5678
m, _ = roundtrip(lambda v: v.lawn_timesync_send(
    t1_us=T1, t2_us=T2, t3_us=T3, exchange_seq=42))
check((m.t1_us, m.t2_us, m.t3_us, m.exchange_seq) == (T1, T2, T3, 42),
      "LAWN_TIMESYNC carries all four timestamps distinctly (ADR-0007)")

m, _ = roundtrip(lambda v: v.lawn_imu_raw_send(
    t_meas_us=BIG, ax=-32768, ay=32767, az=1, gx=-1, gy=2, gz=-3, gap=255))
check(m.ax == -32768 and m.ay == 32767,
      "LAWN_IMU_RAW int16 spans full range")
check(m.gap == 255, "LAWN_IMU_RAW gap indicator survives (ADR-0007)")

m, _ = roundtrip(lambda v: v.lawn_fault_event_send(
    t_meas_us=BIG, code=d.LAWN_FAULT_LINK_DEGRADED,
    nav_state=d.LAWN_NAV_FAULT, latched=1))
check(m.code == d.LAWN_FAULT_LINK_DEGRADED and m.latched == 1,
      "LAWN_FAULT_EVENT survives")

m, _ = roundtrip(lambda v: v.heartbeat_send(
    type=d.MAV_TYPE_GROUND_ROVER, autopilot=d.MAV_AUTOPILOT_GENERIC,
    base_mode=0, custom_mode=0xDEADBEEF, system_status=d.MAV_STATE_ACTIVE))
check(m.custom_mode == 0xDEADBEEF,
      "HEARTBEAT.custom_mode carries a full 32-bit session id (§7.5)")


print("\n== on-wire sizes match IF-0001 §6 ==")

# (name, sender, documented payload bytes)
SPEC = [
    ("HEARTBEAT", lambda v: v.heartbeat_send(
        type=1, autopilot=0, base_mode=0, custom_mode=0xFFFFFFFF,
        system_status=4), 9),
    ("LAWN_DRIVE_CMD", lambda v: v.lawn_drive_cmd_send(
        left_drpm=-1, right_drpm=-1), 4),
    ("LAWN_ARM_CMD", lambda v: v.lawn_arm_cmd_send(magic=0xFFFF, arm=1), 3),
    ("LAWN_STOP_REQ", lambda v: v.lawn_stop_req_send(reason=1), 1),
    ("LAWN_NAV_STATUS", lambda v: v.lawn_nav_status_send(
        t_meas_us=BIG, cmd_age_ms=1, nav_state=1, armed=1, fault=1), 13),
    ("LAWN_WHEEL_STATE", lambda v: v.lawn_wheel_state_send(
        t_meas_us=BIG, left_drpm=-1, right_drpm=-1, left_cmd_drpm=-1,
        right_cmd_drpm=-1, flags=1), 17),
    ("LAWN_LINK_STATS", lambda v: v.lawn_link_stats_send(
        t_meas_us=BIG, frames_accepted=1, frames_rejected=1, dropped_tx=1,
        window_ms=1, heartbeats_missed=1, quality=1), 25),
    ("LAWN_FAULT_EVENT", lambda v: v.lawn_fault_event_send(
        t_meas_us=BIG, code=1, nav_state=1, latched=1), 12),
    ("LAWN_TIMESYNC", lambda v: v.lawn_timesync_send(
        t1_us=BIG, t2_us=BIG, t3_us=BIG, exchange_seq=1), 25),
    ("LAWN_IMU_RAW", lambda v: v.lawn_imu_raw_send(
        t_meas_us=BIG, ax=-1, ay=-1, az=-1, gx=-1, gy=-1, gz=-1, gap=1), 21),
]

sizes = {}
for name, send, doc in SPEC:
    _, raw = roundtrip(send)
    sizes[name] = len(raw)
    check(len(raw) == doc + OVERHEAD,
          f"{name:<18} {len(raw):>3} B on wire (payload {doc} + {OVERHEAD})")

print("\n== IF-0001 §4 maximum frame ==")
worst = max(sizes.values())
check(worst <= 76, f"largest frame {worst} B <= 76 B cap")
print(f"       -> {worst * 10 / 1e6 * 1000:.3f} ms at 1 Mbaud "
      f"(head-of-line floor for any priority scheme)")

print("\n== IF-0001 §6 bandwidth budget ==")
UP = [("HEARTBEAT", 20), ("LAWN_DRIVE_CMD", 20), ("LAWN_TIMESYNC", 5)]
DN = [("HEARTBEAT", 20), ("LAWN_NAV_STATUS", 20), ("LAWN_WHEEL_STATE", 50),
      ("LAWN_LINK_STATS", 1), ("LAWN_TIMESYNC", 5)]
up = sum(sizes[n] * hz for n, hz in UP)
dn = sum(sizes[n] * hz for n, hz in DN)
print(f"       Pi->Pico {up:>5} B/s = {100*up/WIRE_BPS:.2f}%")
print(f"       Pico->Pi {dn:>5} B/s = {100*dn/WIRE_BPS:.2f}%")
check(up == 925, "Pi->Pico budget is 925 B/s as documented")
check(dn == 2592, "Pico->Pi budget is 2592 B/s as documented")
check(abs(100 * (up + dn) / WIRE_BPS - 3.52) < 0.01,
      f"command link total {100*(up+dn)/WIRE_BPS:.2f}% matches documented 3.52%")

imu = sizes["LAWN_IMU_RAW"] * 208
print(f"       LAWN_IMU_RAW @208 Hz = {imu} B/s = {imu*8/1000:.1f} kbit/s "
      f"({100*imu*8/12e6:.2f}% of USB FS)")
check(imu == 6864, "inertial stream is 6864 B/s at 208 Hz")


print("\n== SAF-53: a corrupted frame is rejected whole ==")
_, raw = roundtrip(lambda v: v.lawn_drive_cmd_send(left_drpm=1000,
                                                   right_drpm=1000),
                   COMPID_PI)
bad = bytearray(raw)
bad[11] ^= 0xFF                      # flip a payload bit, leave the CRC alone
rx = MAVLink(io.BytesIO(), srcSystem=SYSID, srcComponent=COMPID_PICO)
rx.robust_parsing = True
got = [m for m in (rx.parse_buffer(bytes(bad)) or [])
       if m.get_type() == "LAWN_DRIVE_CMD"]
check(not got, "payload corruption fails CRC; no LAWN_DRIVE_CMD delivered")

print("\n== IF-0001 §3: identity filtering ==")
mav, buf = encoder(COMPID_PI)
mav.srcComponent = 42                # neither 1 nor 191
mav.lawn_drive_cmd_send(left_drpm=9999, right_drpm=9999)
rx = MAVLink(io.BytesIO(), srcSystem=SYSID, srcComponent=COMPID_PICO)
rx.robust_parsing = True
msgs = [m for m in (rx.parse_buffer(buf.getvalue()) or [])
        if m.get_type() != "BAD_DATA"]
stray = [m for m in msgs if m.get_srcComponent() not in (COMPID_PI, COMPID_PICO)]
check(len(stray) == 1,
      "a frame from compid 42 parses but is identifiable as foreign")
print("       (the receiver must drop it - the parser will not)")


print()
if fails:
    print(f"FAILED: {len(fails)}")
    for f in fails:
        print(f"  - {f}")
    sys.exit(1)
print("All checks passed.")
