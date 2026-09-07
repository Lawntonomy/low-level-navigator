#!/usr/bin/env bash
# motor-sweep.sh -- run the deadband/PWM-speed sweep (TP-0002 CAL-1/CAL-2)
# against the real drivetrain, via diagnostics/motor-probe.
#
# THIS COMMANDS MOTION. The precondition is that both wheels are off the
# ground -- nothing in the firmware checks this, and nothing can: a loaded
# wheel reads SLOWER, so no speed tripwire can detect the difference. You
# must say so explicitly with --wheels-are-off-the-ground; there is no
# default that runs without it.
#
# What it does, in order, failing loudly (not continuing) if any step fails:
#   1. Verify diagnostics/motor-probe/build/motor_probe.uf2 exists locally.
#      This script does not build it -- see diagnostics/motor-probe/README
#      or its CMakeLists.txt for that, it needs a local Pico SDK checkout.
#   2. Copy scripts/enter-bootloader.py and mavlink/python/ to the Pi,
#      mirroring this repo's relative layout (enter-bootloader.py imports
#      ../mavlink/python relative to itself).
#   3. Ask the CURRENTLY RESIDENT firmware to reboot into BOOTSEL, over the
#      command link (scripts/enter-bootloader.py). This only works if that
#      firmware understands LAWN_ENTER_BOOTLOADER -- i.e. it is the main
#      low-level-nav build. If a diagnostic probe is currently flashed
#      instead, this step will time out waiting for the PICOBOOT device, and
#      the fix is a physical BOOTSEL press, not a re-run of this script.
#   4. Wait for 2e8a:000f (PICOBOOT) to enumerate on the Pi.
#   5. Copy motor_probe.uf2 to the Pi and `picotool load -x -v` it.
#   6. Capture the probe's CSV output from /dev/ttyACM0 (its USB CDC; the
#      main low-level-nav firmware has no such port, but this probe does) for
#      long enough to cover the whole sweep, and copy it back.
#
# Usage:
#   scripts/bench/motor-sweep.sh --wheels-are-off-the-ground [options]
#
# Options:
#   --uf2 PATH              motor_probe.uf2 to flash
#                            (default: diagnostics/motor-probe/build/motor_probe.uf2)
#   --cmd-port PATH          command-link device on the Pi (default: /dev/ttyAMA2)
#   --bootsel-wait-seconds N  how long to wait for 2e8a:000f (default: 15)
#   --capture-seconds N      how long to capture /dev/ttyACM0 (default: 60)
#                            The probe: 3 s boot delay, then permille 0..500
#                            step 25 (21 steps) * 1.5 s dwell, then a 0.5 s
#                            stop settle -- about 36 s. 60 s leaves margin for
#                            a slow boot; raise it if the CSV looks truncated.
#
# Env overrides: PI_HOST, PI_HOST_FALLBACK, PI_REMOTE_DIR (scripts/bench/lib/common.sh).
#
# shellcheck disable=SC2088  # the ~ below are deliberate: these strings are
# command lines for the REMOTE shell (built here, expanded there by ssh/
# bench-run.sh), not for this script -- see bench-run.sh's header.
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck source=scripts/bench/lib/common.sh
source "$SCRIPT_DIR/lib/common.sh"
BENCH_RUN="$SCRIPT_DIR/bench-run.sh"

repo_root="$(bench_repo_root)"
cd "$repo_root"

uf2_path="diagnostics/motor-probe/build/motor_probe.uf2"
cmd_port="/dev/ttyAMA2"
bootsel_wait_seconds=15
capture_seconds=60
confirmed=0

while [[ $# -gt 0 ]]; do
    case "$1" in
        --wheels-are-off-the-ground) confirmed=1; shift ;;
        --uf2) uf2_path="$2"; shift 2 ;;
        --cmd-port) cmd_port="$2"; shift 2 ;;
        --bootsel-wait-seconds) bootsel_wait_seconds="$2"; shift 2 ;;
        --capture-seconds) capture_seconds="$2"; shift 2 ;;
        -h|--help) sed -n '2,40p' "$0" | sed 's/^# \{0,1\}//'; exit 0 ;;
        *) echo "motor-sweep: unrecognised argument: $1" >&2; exit 64 ;;
    esac
done

echo "PRECONDITION: both wheels must be off the ground. This sweep drives the"
echo "motors from 0 to 50% duty forward. Nothing in the firmware checks this."
if [[ $confirmed -ne 1 ]]; then
    echo "Refusing to run without --wheels-are-off-the-ground." >&2
    exit 1
fi

if [[ ! -f "$uf2_path" ]]; then
    echo "motor-sweep: $uf2_path does not exist." >&2
    echo "Build it first -- see diagnostics/motor-probe/CMakeLists.txt (needs a" >&2
    echo "local Pico SDK checkout; it is not part of the Docker firmware build)." >&2
    exit 1
fi

echo "motor-sweep: resolving the bench Pi..."
host="$(bench_resolve_host)"
echo "motor-sweep: using $host"

timestamp="$(date -u +%Y%m%dT%H%M%SZ)"
csv_name="motor-sweep-${timestamp}.csv"
mkdir -p bench-data

echo "motor-sweep: [1/6] remote directory"
"$BENCH_RUN" motor-sweep-mkdir "mkdir -p ~/${PI_REMOTE_DIR}"

echo "motor-sweep: [2/6] syncing enter-bootloader.py + mavlink/python"
"$BENCH_RUN" --local motor-sweep-sync \
    "rsync -a --relative scripts/enter-bootloader.py mavlink/python ${host}:~/${PI_REMOTE_DIR}/"

echo "motor-sweep: [3/6] requesting BOOTSEL over the command link ($cmd_port)"
"$BENCH_RUN" motor-sweep-bootsel \
    "~/link-stub-pi/.venv/bin/python3 -u ~/${PI_REMOTE_DIR}/scripts/enter-bootloader.py --port ${cmd_port}"

echo "motor-sweep: [4/6] waiting up to ${bootsel_wait_seconds}s for 2e8a:000f"
if ! "$BENCH_RUN" motor-sweep-wait-bootsel \
    "timeout ${bootsel_wait_seconds} bash -c 'until lsusb | grep -q 2e8a:000f; do sleep 0.3; done'"
then
    echo "motor-sweep: PICOBOOT never enumerated." >&2
    echo "  Most likely cause: the firmware currently on the board does not" >&2
    echo "  speak LAWN_ENTER_BOOTLOADER (e.g. a diagnostic probe is resident" >&2
    echo "  instead of low-level-nav). Press BOOTSEL by hand and re-run, or" >&2
    echo "  reflash low-level-nav first." >&2
    exit 1
fi

echo "motor-sweep: [5/6] flashing motor_probe"
"$BENCH_RUN" --local motor-sweep-scp-uf2 \
    "scp '${uf2_path}' ${host}:~/${PI_REMOTE_DIR}/motor_probe.uf2"
"$BENCH_RUN" motor-sweep-load \
    "~/picotool/picotool load -x -v ~/${PI_REMOTE_DIR}/motor_probe.uf2"

echo "motor-sweep: [6/6] capturing ${capture_seconds}s of /dev/ttyACM0 to $csv_name"
# `timeout` cutting the capture off (exit 124) is the expected way this ends,
# not a failure -- the probe runs to completion and then idles forever
# (tight_loop_contents()) printing nothing more, so something has to stop it.
capture_cmd="timeout ${capture_seconds} cat /dev/ttyACM0 > ~/${PI_REMOTE_DIR}/${csv_name}"
# shellcheck disable=SC2016  # single-quoted on purpose: $ec/$? are for the
# REMOTE shell to expand when it runs this, not this script.
capture_cmd+='; ec=$?; if [ "$ec" -eq 124 ]; then exit 0; else exit "$ec"; fi'
"$BENCH_RUN" motor-sweep-capture "$capture_cmd"

echo "motor-sweep: copying CSV back"
"$BENCH_RUN" --local motor-sweep-scp-csv \
    "scp ${host}:~/${PI_REMOTE_DIR}/${csv_name} bench-data/${csv_name}"

echo "motor-sweep: done. CSV: $repo_root/bench-data/${csv_name}"
echo "motor-sweep: logs under $repo_root/bench-logs/ (one per step above)."
