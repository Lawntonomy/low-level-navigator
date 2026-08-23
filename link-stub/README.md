# Inter-tier link stub

Bring-up code for the RP2350 ↔ Raspberry Pi link. Exercises the decisions in
[IF-0001](../../system-design/interfaces/inter-tier-protocol.md) (protocol) and
[TP-0001](../../system-design/test-plans/0001-inter-tier-link-bringup.md) (transport) on real
hardware.

**This is not the navigator.** No control loop, no PID, no PWM; the wheel speeds are synthetic and
the drive requests are a sine wave. Its only job is to make the link's behaviour measurable before
either tier is written for real.

## Wiring

| | Pico | Pi | Baud |
| --- | --- | --- | --- |
| **Command link** | `uart0` — GPIO 0 (TX), GPIO 1 (RX) | `uart2` — GPIO 0 (RX), GPIO 1 (TX) | 1,000,000 |
| **Console** | `uart1` — GPIO 20 (TX) | *USB-serial adapter to a laptop* | 115,200 |
| **Scope** | GPIO 22 | — | — |

TX↔RX cross over, and **grounds must be common** — ADR-0003 flags this as "to be checked rather
than assumed", and TP-0001 T1.4 is that check.

The console goes to a USB-serial adapter rather than the Pi on purpose: it stays readable while the
Pi is rebooting, wedged, or unplugged, which is when you most want it.

On the Pi, add to `/boot/firmware/config.txt` (Bookworm; `/boot/config.txt` on older) and reboot:

```
dtoverlay=uart2
```

Nothing enables `uart2` by default. See TP-0001 T0.1, and T0.3 for why you should not trust the
`ttyAMA*` number.

## Build and run

**Regenerate the MAVLink bindings** (only needed if the dialect changed — output is committed):

```bash
python3 -m venv .venv && .venv/bin/pip install pymavlink
PYTHON=.venv/bin/python ./generate.sh
```

**Pico:**

```bash
cd pico && mkdir -p build && cd build
cmake .. -DPICO_SDK_PATH=$HOME/Documents/pico-sdk -DPICO_BOARD=pico2 && make -j4
```

Flash `build/link_stub.uf2` by holding BOOTSEL. Watch the console at 115200 8N1 — it prints the
achieved baud, the ppm error, and the session id before anything else runs.

**Pi:**

```bash
pip install pyserial
./pi/link_stub.py --port /dev/ttyAMA1
```

**Without hardware**, the dialect and every documented number can still be checked:

```bash
./pi/selftest.py
```

## What each part demonstrates

| Decision | Where |
| --- | --- |
| D1 three channels, `uart0` command / `uart1` console / USB bulk | wiring above |
| D2 1 Mbaud, achieved rate asserted at boot | `pico/main.c` baud check — refuses to run past 2% error |
| D3 no hardware flow control | `uart_set_hw_flow(..., false, false)`; pyserial `rtscts=False` |
| D4 64-bit µs timestamps | `time_us_64()` throughout; selftest proves the field is really 64-bit |
| D5 stdio off, drop-on-full TX ring | `pico_enable_stdio_*(0)`; `tx_push()` drops whole frames and counts them |
| D6 RX pull-up for RP2350-E9 | `gpio_pull_up(CMD_RX_PIN)` |
| §7.2 drain-to-newest | `pending_drive` collapses a burst to its newest request |
| §7.4 four-timestamp TIMESYNC | `ClockModel` fits offset **and** skew, min-filtered on delay |
| §7.5 stall vs restart | session id in `HEARTBEAT.custom_mode` |
| SAF-53 reject frames whole | selftest corrupts a payload and proves nothing is delivered |

## Things to try

```bash
./pi/link_stub.py --port /dev/ttyAMA1 --no-arm       # SAF-11: it must not move
./pi/link_stub.py --port /dev/ttyAMA1 --stall 0.3    # brief: ramp to zero, stays armed
./pi/link_stub.py --port /dev/ttyAMA1 --stall 3.0    # long: disarms, needs re-arm
```

Then pull the command wire mid-run, and restart the Pi-side process — the second one changes the
session id, so the Pico should report `peer RESTART` and disarm rather than resuming.

## Known limitations, deliberately

- **`t3_us` is stamped in software, not at the last byte out.** The stub drains the ring and blocks
  for one frame (~370 µs) to get close. IF-0001 §7.4 wants PIO edge capture; TP-0001 T4.1 specifies
  it. **Everything the stub reports about sync precision is bounded by this**, so do not read the
  offset figure as the achievable one.
- **`t2_us` is the FIFO-read timestamp of the frame's first byte**, not the start-bit edge — good to
  roughly the FIFO trigger jitter (~±25 µs), not the ~±2 µs the pin-edge path gives.
- **Bare-metal, no FreeRTOS.** TP-0001 Phase 2 tests loop-period independence under a scheduler;
  this cannot stand in for that.
- **Synthetic wheel data.** There is no encoder behind `LAWN_WHEEL_STATE`.
- The Pico applies a magnitude clamp only — no slew limit, no direction interlock. `SAF-30` and
  `SAF-31` are not represented.
