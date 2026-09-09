# scripts/bench/

Every command that reaches the bench hardware -- the Pi, the Pico, the motors -- goes through a
script in this directory, and every run is logged under `bench-logs/`. This exists because before
it did, hardware interaction was a series of ad-hoc `ssh andrew@raspberrypi.local '...'`
one-liners: reviewable by no one, and re-runnable only by whoever remembered exactly what they
typed.

**The standing rule: if the script you need does not exist yet, write it, commit it, and only
then run it.** A command composed in the moment and pasted into a transcript is an anecdote. A
script in this directory, reviewed before it runs, is not.

## What's here

- **`bench-run.sh`** -- the wrapper everything else uses. Runs one command (over ssh on the Pi by
  default, or locally with `--local`) and logs the exact command, target, start time, full
  stdout+stderr, exit status and duration to `bench-logs/<UTC timestamp>-<label>.log`, in addition
  to echoing to the terminal. Refuses any command containing `sudo` outright (see below).

- **`motor-sweep.sh`** -- TP-0002 CAL-1/CAL-2's deadband and PWM-speed sweep, via
  `diagnostics/motor-probe`. **Commands motion.** Requires
  `--wheels-are-off-the-ground` explicitly; there is no default that drives the motors without it,
  because nothing in the firmware can check that precondition and a loaded wheel reads *slower*,
  not stalled, so no speed limit would catch the mistake either.

- **`cal0-check.sh`** -- flashes the main `low-level-nav` firmware and streams
  `LAWN_WHEEL_STATE` from the command link (`scripts/bench/pi/wheel_state_monitor.py`) so TP-0002
  CAL-0's acceptance criterion -- a stopped wheel reads zero and clears its VALID flag within
  `t_stale` -- can be watched. Does not command motion (see the script's header for why that is
  currently true, and when to re-check it).

- **`link-status.sh`** -- read-only: USB enumeration, `picotool info`, a read of the Pico's CDC,
  and a passive listen on the command link. Sends nothing to the board. Run it before any flash,
  because what it finds decides *how* to flash: telemetry on the link means a `low-level-nav` build
  is resident and `cal0-check.sh` can reflash it; a `2e8a` device with no telemetry means a probe is
  resident and `picotool load -f` works directly; neither means BOOTSEL.

- **`imu-probe.sh`** -- flashes `diagnostics/imu-probe` and reads its report, answering
  `research/imu-driver-findings.md` §7 on hardware: LSM6DSOX on the bus at GP12/13, `WHO_AM_I`,
  and whether INT1 on GP14 produces data-ready edges. Does not command motion. Tries
  `picotool load -f` first and falls back to `enter-bootloader.py`.

- **`restore-lln.sh`** -- puts `low-level-nav` back after a probe run. Distinct from
  `cal0-check.sh`, which reaches the bootloader by asking the resident firmware over the command
  link and so requires that firmware to already *be* `low-level-nav`; this covers the other
  direction, where a probe is resident and its USB stdio makes `picotool load -f` work directly.

- **`lib/common.sh`** -- shared host/path config and the sudo guard, sourced by the scripts above.
  Not run directly.

- **`pi/wheel_state_monitor.py`** -- runs ON THE PI (opens a local serial device); copied there by
  `cal0-check.sh`, not invoked directly from here.

- **`pi/link_listen.py`** -- also runs ON THE PI; copied there by `link-status.sh`. A passive
  command-link listener that transmits nothing, so it cannot arm or move the machine. Deliberately
  dependency-free -- no pymavlink, no generated dialect -- which costs it checksum validation
  (`CRC_EXTRA` is per-message and lives in the dialect it avoids needing), so a stray byte can
  invent a frame. That shows up as a one-off count against an implausible message id, easy to read
  past when real traffic is arriving at a steady 20-50 Hz.

## Preconditions, every time

- Reachability: `andrew@raspberrypi.local`, falling back to `andrew@192.168.10.230` if mDNS is
  slow (`PI_HOST`/`PI_HOST_FALLBACK` env vars override either).
- `andrew` is in `dialout`; serial ports need no `sudo`. **`sudo` needs an interactive password
  nobody has and must never be attempted** -- `bench-run.sh` refuses any command containing the
  string `sudo`, on purpose, rather than letting it hang at an unanswerable prompt on a machine
  with motors attached. If something genuinely needs root, ask Andrew to run it directly.
  Auth failures show up as ssh giving up quickly, not as a hang.
- `picotool` is at `~/picotool/picotool` on the Pi, not on `$PATH`.
- The command link is `/dev/ttyAMA2` at 1 Mbaud. `/dev/ttyACM0` (the Pico's own USB CDC) only
  exists while a **diagnostic** build (e.g. `motor_probe`) is resident; the main `low-level-nav`
  build has USB stdio off and presents no such port.
- Getting the resident firmware into BOOTSEL over the command link
  (`scripts/enter-bootloader.py`) only works if that firmware speaks `LAWN_ENTER_BOOTLOADER` --
  i.e. it is already a `low-level-nav` build. If a diagnostic probe with no such handling is
  currently flashed, both `motor-sweep.sh` and `cal0-check.sh` will time out waiting for
  `2e8a:000f` and say so; the fix is a physical BOOTSEL press, not a re-run.
- Remote Python needs `python3 -u` (stdout is fully buffered over a non-TTY ssh pipe) and the
  `pymavlink`/`pyserial` venv at `~/link-stub-pi/.venv` on the Pi.
- Scripts that copy Python to the Pi mirror this repo's relative layout underneath
  `~/lln-bench/` (`PI_REMOTE_DIR`) -- `enter-bootloader.py` imports `../mavlink/python` relative
  to itself, and `wheel_state_monitor.py` imports the same thing three directories up, so both
  only resolve if `mavlink/python/` lands in the corresponding place alongside them.

## `bench-logs/` is gitignored, on purpose

Every run still writes a full log to `bench-logs/` locally, with the timestamp, command, output,
exit status and duration -- Andrew can see exactly what ran and when, right after it runs, without
needing anything committed. It is not tracked in git:

- It is exactly analogous to `build/`, `build-fw/`, `build-local/`, `build-test/` -- all already
  gitignored in this repo -- generated output from running a script, not a script itself.
- What makes a run reproducible is the **committed script**, not a copy of one run's raw output.
  Keeping every historical capture in git history is a one-way ratchet: history never shrinks, and
  a repeated CAL-0 stream or a long motor-sweep capture is not small.
- Raw serial capture is not source and does not benefit from diffing; it benefits from being read
  once, near the run, which a local file supports as well as a committed one.

**If Andrew disagrees, this is a one-line reversal** -- drop the `bench-logs/` entry from
`.gitignore` and `git add -f` the directory. A particular run worth preserving permanently (e.g.
the run that closes a CAL-0 acceptance check) can always be committed individually with `git add
-f bench-logs/<that-one>.log`, or copied into `system-design/` as recorded evidence, without
changing the default for every other run.

`bench-data/` (captured CSVs, e.g. from `motor-sweep.sh`) follows the same reasoning and is
gitignored the same way.

## Adding a new bench script

1. Write it here, using `bench-run.sh` for every step that reaches the Pi or the Pico.
2. Make the precondition it depends on explicit in a comment and, if it is safety-relevant (can
   move something), require an explicit confirmation flag rather than a default that runs.
3. Fail loudly and specifically at each step -- a script that presses on after a failed step and
   produces confusing output two steps later is worse than one that stops immediately.
4. Commit it.
5. Only then run it against hardware.
