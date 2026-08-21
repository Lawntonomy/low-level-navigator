# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## What this is

Firmware for the "low-level navigator" board of Lawntonomy, an autonomous lawnmower. It runs on a
Raspberry Pi Pico 2 (RP2350, `PICO_BOARD=pico2`) under FreeRTOS and closes the motor control loop:
read quadrature encoders via PIO, run a PID controller per wheel, and drive motor PWM via PIO. A
higher-level system (not in this repo) is expected to send guidance/navigation commands to this
board; this board is only responsible for turning those into wheel motion.

## Build

Build happens inside Docker (the container has the ARM toolchain, Pico SDK, picotool, and
FreeRTOS-Kernel pinned to known-good checkouts):

```bash
docker build -t low-level-navigator .
docker run --rm -v $(pwd):/workspace low-level-navigator
```

This produces `build/low-level-nav.uf2` (flash by holding BOOTSEL and copying the file to the
Pico's mass-storage drive), plus `.elf`/`.bin`/`.hex`/`.map`.

To build outside Docker (e.g. with local SDK checkouts), use `scripts/build.sh`, which expects
sibling checkouts at `../../pico-sdk`, `../../pico-tool`, and `../../FreeRTOS-Kernel`, then runs:

```bash
cmake .. -DPICO_SDK_PATH=$PICO_SDK_PATH -DPICO_BOARD=pico2 \
         -Dpicotool_DIR=$PICO_TOOL_PATH -DFREERTOS_KERNEL_PATH=$FreeRTOS_PATH \
         -DCMAKE_EXPORT_COMPILE_COMMANDS=1 && make
```

## Test

Host-side unit tests live in `test/` as a **separate CMake project** — host compiler, no Pico SDK,
no ARM toolchain, no `pico_sdk_init()`. Only hardware-independent logic is compiled in.

```bash
./scripts/test.sh
```

First run downloads googletest via CMake FetchContent (needs network); later runs use the copy
cached in `build-test/`. To run one test or one suite:

```bash
./build-test/lln_tests --gtest_filter='PidClass.ProportionalResponseToPositiveError'
```

Tests named `DISABLED_*` document known defects: each asserts the behavior the code *should* have
and currently fails. Run them with `./build-test/lln_tests --gtest_also_run_disabled_tests`. When
fixing one of these bugs, drop the `DISABLED_` prefix so the test becomes the regression guard.

Production code that needs to be testable must not depend on the Pico SDK or FreeRTOS in its
header. `test/stubs/logger_stub.cpp` provides a host implementation of `Log::` so code under test
can keep its logging calls; add similar stubs rather than stripping calls out of production code.

The `include/googletest` directory is empty and the `enable_testing()`/`gtest_discover_tests()`
lines in the firmware [CMakeLists.txt](CMakeLists.txt) are commented out — both are dead
scaffolding from an earlier attempt, superseded by `test/`.

## Verification expectations

Claude cannot run this firmware: there is no emulator and no hardware in the loop. When reporting
on a change, state explicitly which of these applies rather than saying "done":

- **Unit tested** — covered by a test in `test/` that was actually run.
- **Compiles only** — built, but behavior unverified.
- **Not verified** — needs bench testing on hardware. Say what to watch on the UART.

Note that `docker` currently requires group membership the user does not have, so even the firmware
build may not be runnable from a Claude session — do not claim a successful build without one.

## Lint

CI runs `clang-format -style=file` (config in [.clang-format](.clang-format)) over every
`.cpp/.hpp/.cu/.c/.h` file and fails the build on any diff. Run the same check locally before
pushing:

```bash
find . -regex '.*\.\(cpp\|hpp\|cu\|c\|h\)' -exec clang-format -style=file -i {} \;
```

`.clang-tidy` enforces naming conventions: `PascalCase` classes/namespaces/enums, `camelCase`
functions/parameters, `snake_case` variables, `UPPER_CASE` globals.

## Architecture

- **`src/hardware_drivers/`** — thin, direct-register/PIO drivers with no business logic:
  `encoder.cpp/.pio` (quadrature decode via PIO, `get_left_rpm`/`get_right_rpm`), `pwm.cpp/.pio`
  (PIO-based PWM for motor drive), `ws2812.pio` (status NeoPixel), `gpio_defines.h` (single source
  of truth for pin assignments and constants like `pwm_frequency`/`encoder_ticks`).
- **`src/high_level_drivers/`** — hardware-agnostic control logic: `pid.cpp/.hpp` (a per-wheel
  `PidClass` with configurable output clamping) and `navigator.cpp/.hpp`, which is currently a stub
  — the state-machine design described below and in [README.md](README.md) is not implemented yet.
- **`src/utility/logger.*`** — a `Log` static class intended to run as a FreeRTOS task
  (`logger_task`) with `trace`/`info`/`warn`/`debug`/`error` levels.
- **`src/low-level-navigator.cpp`** — current `main()`. This is a flat, pre-state-machine control
  loop (no FreeRTOS tasks yet): init GPIO/PIO, run two `PidClass` instances closed over encoder RPM
  vs. a hardcoded `left_target`/`right_target`, write PWM, and flip direction GPIOs when a wheel
  crosses zero RPM. Treat this file as the integration point that the state machine below is meant
  to replace.

### Intended state machine (design target, not yet built)

Per [README.md](README.md), the navigator is meant to be a state machine:

1. **Pre-Calibration Idle** — no commands, zero speed, no valid calibration.
2. **Calibration** — running the calibration process.
3. **Idle Navigation** — valid calibration, no guidance commands.
4. **Active Guidance** — actively executing guidance commands.
5. **Exiting Guidance** — no new guidance commands, still executing in-flight ones, ramping to 0.

A related known gap (see comment in [navigator.cpp](src/high_level_drivers/navigator.cpp)): wheel
direction must never jump straight from forward to backward. Transitions should go
`forward -> stopped -> backward` (and the reverse), never directly across zero.

### Pin/hardware notes

Pin assignments live in [gpio_defines.h](src/hardware_drivers/gpio_defines.h) as the
`gpio::pins` enum — check there before wiring new hardware. UART is on GPIO 0/1
([docs/config1.md](docs/config1.md)); the default stdio UART pins are overridden to 16/17 in
[CMakeLists.txt](CMakeLists.txt).

## Task tracking

Work is tracked as GitHub Issues on `Lawntonomy/low-level-navigator`. Use the `gh` CLI
(`gh issue list`, `gh issue view <n>`, `gh issue create`) rather than a local backlog file.
