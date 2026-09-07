#!/usr/bin/env bash
# cal0-check.sh -- flash the main firmware and stream LAWN_WHEEL_STATE from
# the command link, timestamped, so TP-0002 CAL-0's acceptance criterion (a
# stopped wheel reads zero and clears its VALID flag within t_stale) can be
# watched by a human.
#
# Does NOT command motion. The main low-level-nav firmware cannot drive the
# motors at all as of this writing -- control_task calls
# motors::safe_state() unconditionally (see diagnostics/motor-probe/main.cpp's
# header comment) -- so there is no wheels-off-the-ground precondition here.
# That claim is about the CURRENT firmware; re-check it if control_task grows
# the ability to drive before this script's assumption does.
#
# What it does, in order, failing loudly (not continuing) if any step fails:
#   1. Verify the firmware image exists locally.
#   2. Copy scripts/enter-bootloader.py, mavlink/python/, and
#      scripts/bench/pi/wheel_state_monitor.py to the Pi, mirroring this
#      repo's relative layout.
#   3. Ask the CURRENTLY RESIDENT firmware to reboot into BOOTSEL, over the
#      command link. Only works if that firmware speaks
#      LAWN_ENTER_BOOTLOADER (i.e. it is already a low-level-nav build); if a
#      diagnostic probe is resident instead, this times out and needs a
#      physical BOOTSEL press.
#   4. Wait for 2e8a:000f (PICOBOOT) to enumerate.
#   5. Copy the firmware to the Pi and `picotool load -x -v` it.
#   6. Stream and decode LAWN_WHEEL_STATE from the command link with
#      scripts/bench/pi/wheel_state_monitor.py, printing to the terminal AND
#      into the bench-run log, for --seconds.
#
# --seconds is a fixed window, not "run until Ctrl-C": a local Ctrl-C tears
# down the ssh connection, and the remote python then gets a SIGHUP on
# session hangup, not the SIGINT its KeyboardInterrupt handler is written
# for -- so it would exit without its cleanup line, and there is no live
# process left to watch anyway. Pick a --seconds long enough to watch and
# re-run for another window rather than trying to interrupt one, or run
# wheel_state_monitor.py directly on the Pi if you need genuinely
# open-ended streaming.
#
# Usage:
#   scripts/bench/cal0-check.sh [options]
#
# Options:
#   --uf2 PATH               firmware to flash (default: build/low-level-nav.uf2)
#   --cmd-port PATH          command-link device on the Pi (default: /dev/ttyAMA2)
#   --bootsel-wait-seconds N how long to wait for 2e8a:000f (default: 15)
#   --seconds N              how long to stream (default: 30)
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

uf2_path="build/low-level-nav.uf2"
cmd_port="/dev/ttyAMA2"
bootsel_wait_seconds=15
seconds=30

while [[ $# -gt 0 ]]; do
    case "$1" in
        --uf2) uf2_path="$2"; shift 2 ;;
        --cmd-port) cmd_port="$2"; shift 2 ;;
        --bootsel-wait-seconds) bootsel_wait_seconds="$2"; shift 2 ;;
        --seconds) seconds="$2"; shift 2 ;;
        -h|--help) sed -n '2,48p' "$0" | sed 's/^# \{0,1\}//'; exit 0 ;;
        *) echo "cal0-check: unrecognised argument: $1" >&2; exit 64 ;;
    esac
done

if [[ ! -f "$uf2_path" ]]; then
    echo "cal0-check: $uf2_path does not exist." >&2
    echo "Build it first: docker run --rm -v \"\$(pwd)\":/workspace low-level-navigator" >&2
    exit 1
fi

echo "cal0-check: resolving the bench Pi..."
host="$(bench_resolve_host)"
echo "cal0-check: using $host"

echo "cal0-check: [1/5] remote directory"
"$BENCH_RUN" cal0-mkdir "mkdir -p ~/${PI_REMOTE_DIR}/scripts/bench/pi"

echo "cal0-check: [2/5] syncing enter-bootloader.py, wheel_state_monitor.py, mavlink/python"
"$BENCH_RUN" --local cal0-sync \
    "rsync -a --relative scripts/enter-bootloader.py scripts/bench/pi/wheel_state_monitor.py mavlink/python ${host}:~/${PI_REMOTE_DIR}/"

echo "cal0-check: [3/5] requesting BOOTSEL over the command link ($cmd_port)"
"$BENCH_RUN" cal0-bootsel \
    "~/link-stub-pi/.venv/bin/python3 -u ~/${PI_REMOTE_DIR}/scripts/enter-bootloader.py --port ${cmd_port}"

echo "cal0-check: waiting up to ${bootsel_wait_seconds}s for 2e8a:000f"
if ! "$BENCH_RUN" cal0-wait-bootsel \
    "timeout ${bootsel_wait_seconds} bash -c 'until lsusb | grep -q 2e8a:000f; do sleep 0.3; done'"
then
    echo "cal0-check: PICOBOOT never enumerated." >&2
    echo "  Most likely cause: the firmware currently on the board does not" >&2
    echo "  speak LAWN_ENTER_BOOTLOADER (e.g. a diagnostic probe is resident" >&2
    echo "  instead of low-level-nav). Press BOOTSEL by hand and re-run, or" >&2
    echo "  reflash low-level-nav first." >&2
    exit 1
fi

echo "cal0-check: [4/5] flashing $uf2_path"
"$BENCH_RUN" --local cal0-scp-uf2 \
    "scp '${uf2_path}' ${host}:~/${PI_REMOTE_DIR}/low-level-nav.uf2"
"$BENCH_RUN" cal0-load \
    "~/picotool/picotool load -x -v ~/${PI_REMOTE_DIR}/low-level-nav.uf2"

echo "cal0-check: giving the board a moment to boot"
sleep 3

echo "cal0-check: [5/5] streaming LAWN_WHEEL_STATE from $cmd_port"
echo "cal0-check: watch: a stopped wheel should read drpm=0 and its VALID bit"
echo "cal0-check: should clear within t_stale (informational default printed"
echo "cal0-check: below; see wheel_state_monitor.py's header for the caveat)."
monitor_cmd="~/link-stub-pi/.venv/bin/python3 -u ~/${PI_REMOTE_DIR}/scripts/bench/pi/wheel_state_monitor.py --port ${cmd_port} --seconds ${seconds}"
"$BENCH_RUN" cal0-stream "$monitor_cmd"
