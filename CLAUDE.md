# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## What this is

Firmware for the low-level tier of Lawntonomy, an autonomous lawnmower. It runs on a Raspberry Pi
Pico 2 (RP2350, `PICO_BOARD=pico2`) under FreeRTOS and owns **everything that can move the
machine**: motor drive, encoder capture, inertial sensing, arming, and the failsafe.

A Raspberry Pi runs planning and issues navigation *requests* over a MAVLink link. Per ADR-0001 the
low-level tier is independently safe: it does not assume the high-level tier is alive, correct, or
timely, and the Pi reboots as a *routine* event during image updates.

**The design authority is a separate repository**, normally checked out alongside this one at
`../system-design`:

- `adr/` — accepted decision records. Immutable; a new ADR supersedes rather than edits.
- `requirements/` — `SAF-*` and `NAV-*`, each with current status.
- `interfaces/inter-tier-protocol.md` — **IF-0001**, the wire protocol this firmware implements.

Read the relevant records before changing behaviour. Reads outside this repo may prompt.

## Build

**Local build (preferred — fast, no container):**

```bash
cmake -S . -B build-local \
  -DPICO_SDK_PATH=$HOME/Documents/pico-sdk \
  -DFREERTOS_KERNEL_PATH=$HOME/Documents/FreeRTOS-Kernel \
  -DPICO_BOARD=pico2 -DCMAKE_BUILD_TYPE=Debug
cmake --build build-local -j"$(nproc)"
```

Produces `build-local/low-level-nav.uf2` plus `.elf`/`.bin`/`.map`.

Docker remains available (`docker build -t low-level-navigator . && docker run --rm -v $(pwd):/workspace low-level-navigator`)
but is **not** usable by the current user, who is not in the `docker` group. Prefer the local build.

`scripts/build.sh` has stale relative paths (`../../pico-sdk`) from before this repo moved under
`Lawntonomy/`. Use the command above rather than the script until it is fixed.

## Test

Host-side unit tests live in `test/` as a **separate CMake project** — host compiler, no Pico SDK,
no ARM toolchain, no `pico_sdk_init()`. Only hardware-independent logic compiles in.

```bash
./scripts/test.sh
./build-test/lln_tests --gtest_filter='PidClass.*'          # one suite
./build-test/lln_tests --gtest_also_run_disabled_tests      # the known-defect set
```

First run fetches googletest (needs network); later runs use the cache in `build-test/`.

Tests named `DISABLED_*` document **known, unfixed defects**: each asserts the behaviour the code
*should* have and currently fails. When fixing one, drop the prefix so the test becomes the
regression guard. See issues #12 and #13.

Production code that must be testable may not depend on the Pico SDK or FreeRTOS *in its header*.
`test/stubs/logger_stub.cpp` provides a host `Log::` so code under test keeps its logging calls;
add stubs rather than stripping calls from production code.

## Verification expectations

There is no emulator and no hardware in the loop from a Claude session. State which of these
applies rather than saying "done":

- **Unit tested** — covered by a test in `test/` that was actually run.
- **Compiles only** — built via the local build above. This *is* now achievable; don't claim it
  without running it.
- **Not verified** — needs bench testing. Say what to watch on the UART.

## Lint

CI runs `clang-format -style=file` over every `.cpp/.hpp/.cu/.c/.h` and fails on any diff.
**`mavlink/` is vendored and excluded** — do not reformat it.

```bash
git ls-files -- 'src/*' 'test/*' 'link-stub/*' | grep -E '\.(cpp|hpp|c|h)$' \
  | xargs clang-format -style=file -i
```

Use `git ls-files` rather than `find`: local build directories under `link-stub/pico/build/`
contain generated SDK headers that will never satisfy the format check. They are git-ignored, so
CI never sees them, but a bare `find` will report them as failures.

`.clang-tidy` sets naming: `PascalCase` types/namespaces, `camelCase` functions/parameters,
`snake_case` variables, `UPPER_CASE` globals.

## Architecture

**`src/main.cpp`** — the FreeRTOS task foundation and `main()`. Creates `control`, `link_rx`,
`link_tx`, `telem`, and `logger` tasks via `must_create`, which **halts rather than continuing** if
a task cannot be created — firmware that silently boots without its control task is worse than
firmware that refuses to boot. Also holds the stack-overflow hook. The watchdog is armed last so a
slow boot cannot trip it before the control task exists.

**`src/app/`** — application layer, and where the safety argument lives:

- `rt.h` — the real-time structure in one place: every task priority, stack size, core affinity and
  period, each with its reason. Change timing here, not scattered through call sites.
- `safety.{hpp,cpp}` — arming, command freshness, fault state. **Owns every decision the high-level
  tier is not allowed to make** (`SAF-33`, IF-0001 §8).
- `link.{hpp,cpp}` — MAVLink transport for the command link. Deliberately does **not** expose
  MAVLink types outward, so the control path never depends on wire representation.
- `log.{hpp,cpp}` — non-blocking console logging. Per ADR-0003 logging is diagnostic and must never
  be required for safe operation, nor block the control loop.
- `board.h` — link and instrumentation pins. Motor/encoder/NeoPixel pins stay in
  `hardware_drivers/gpio_defines.h` so the two do not tangle.

**`src/hardware_drivers/`** — thin PIO/DMA drivers, no business logic. `encoder.cpp/.pio`,
`pwm.cpp/.pio`, `ws2812.pio`, `gpio_defines.h`. PIO headers are generated by
`pico_generate_pio_header` in [CMakeLists.txt](CMakeLists.txt).

Two hardware facts that are **not** obvious from the code and have already caused wrong
documentation once:

> **The encoders are not quadrature.** `encoder.pio` is a single-pin period counter — one GPIO per
> side, counting PIO cycles between high→low transitions into a DMA ring buffer.
> `get_left_rpm`/`get_right_rpm` return an **unsigned magnitude**; direction is *not measurable*.
> Fixing that needs a second channel per encoder, not a firmware change. See issue #12 — a stalled
> wheel also reports its pre-stall speed indefinitely, because DREQ pacing means the DMA simply
> stops rather than writing zeros.

> **`pwm.pio` raises its pin only when `X == Y`** while counting Y down from the period, so a level
> **above** the period produces **0% duty, not 100%**. Always clamp to the period before writing.

**`src/high_level_drivers/`** — `pid.{cpp,hpp}` (per-wheel `PidClass`, host-testable) and
`navigator.{cpp,hpp}`, still a stub.

**`mavlink/`** — generated bindings for the Lawntonomy dialect: `c/` for firmware, `python/` for the
Pi and bench. **Generated, vendored, and excluded from lint** — regenerate with the pinned
`pymavlink`, never hand-edit.

**`link-stub/`** — minimal bring-up stubs for both tiers (`pico/`, `pi/`), used to validate the
transport independently of the firmware. `pi/selftest.py` verifies frame sizes.

## Task tracking

GitHub Issues on `Lawntonomy/low-level-navigator` via `gh` (`gh issue list/view/create/comment`),
not a local backlog file. Cite `SAF-*` IDs from `../system-design/requirements/` where a change
bears on one.

## Subagents

`.claude/agents/` defines five reviewers — `firmware-reviewer`, `safety-reviewer`,
`architecture-advisor`, `hardware-researcher`, `linux-platform-researcher`. **They only register
when the session's working directory is this repository**, not the `Lawntonomy/` umbrella
directory. Working from the umbrella means passing their personas to a general-purpose agent
inline.
