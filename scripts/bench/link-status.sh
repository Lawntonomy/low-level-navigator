#!/usr/bin/env bash
# link-status.sh -- what is on the Pico right now? Read-only.
#
# Sends nothing to the board: it cannot arm it, move it, or reflash it. Run it
# before any flash, because it answers the question that decides HOW to flash:
#
#   - Telemetry on the command link  => a low-level-nav build is resident, so
#     it speaks LAWN_ENTER_BOOTLOADER and cal0-check.sh can reflash it. Check
#     the armed line first; enter-bootloader.py is refused while armed.
#   - A 2e8a USB device and /dev/ttyACM* but NO telemetry => a diagnostic probe
#     with USB stdio is resident. `picotool load -f` works directly and no
#     BOOTSEL press is needed. This is what imu-probe.sh and restore-lln.sh
#     rely on.
#   - Neither => the board is off, the port is wrong, or a no-stdio image is
#     resident and stalled. That last case is the only one needing BOOTSEL.
#
# Usage:
#   scripts/bench/link-status.sh [--seconds N] [--cmd-port PATH] [--cdc PATH]
#
# Env overrides: PI_HOST, PI_HOST_FALLBACK, PI_REMOTE_DIR (lib/common.sh).
#
# shellcheck disable=SC2088  # the ~ below are for the REMOTE shell, expanded
# there by bench-run.sh, not here.
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck source=scripts/bench/lib/common.sh
source "$SCRIPT_DIR/lib/common.sh"
BENCH_RUN="$SCRIPT_DIR/bench-run.sh"

seconds=5
cmd_port=/dev/ttyAMA2
cdc=/dev/ttyACM0
cdc_seconds=12
baud=1000000

while [[ $# -gt 0 ]]; do
    case "$1" in
        --seconds) seconds="$2"; shift 2 ;;
        --cmd-port) cmd_port="$2"; shift 2 ;;
        --cdc) cdc="$2"; shift 2 ;;
        *) echo "link-status: unknown option $1" >&2; exit 64 ;;
    esac
done

repo_root="$(bench_repo_root)"
cd "$repo_root"
host="$(bench_resolve_host)"

echo "link-status: [1/4] usb enumeration"
"$BENCH_RUN" link-status-usb \
    "lsusb | grep -i 2e8a || echo '(no Raspberry Pi USB device)'"
"$BENCH_RUN" link-status-cdc-dev \
    "ls -l ${cdc} 2>/dev/null || echo '(no CDC device)'"
"$BENCH_RUN" link-status-picotool \
    "~/picotool/picotool info 2>&1 | head -20"

# A probe image and a stalled no-stdio image both look silent on the command
# link. Reading the CDC tells them apart, and says WHICH probe -- worth knowing
# before reflashing over someone else's experiment. Reading is passive; the
# probes only print.
#
# `timeout -s INT ... cat` never exits 0 (cat has no end), so its status says
# nothing about whether bytes arrived. Test for the device separately and let
# an empty read speak for itself.
echo "link-status: [2/4] reading ${cdc} for ${cdc_seconds}s"
"$BENCH_RUN" link-status-cdc-read \
    "if [ -e ${cdc} ]; then timeout -s INT ${cdc_seconds} cat ${cdc}; echo '[end of CDC read; empty means the resident image printed nothing]'; else echo '(no CDC device)'; fi"

echo "link-status: [3/4] syncing the listener"
"$BENCH_RUN" --local link-status-sync \
    "rsync -a --relative scripts/bench/pi/link_listen.py ${host}:~/${PI_REMOTE_DIR}/"

# link_listen.py needs pyserial only -- no pymavlink, no generated dialect.
# The link-stub venv already has it and is what cal0-check.sh uses.
echo "link-status: [4/4] listening on ${cmd_port} for ${seconds}s (transmitting nothing)"
"$BENCH_RUN" link-status-listen \
    "timeout -s INT $((seconds + 10)) ~/link-stub-pi/.venv/bin/python3 -u ~/${PI_REMOTE_DIR}/scripts/bench/pi/link_listen.py --port ${cmd_port} --baud ${baud} --seconds ${seconds}" ||
    echo "link-status: no telemetry -- see the usb section above before concluding anything"
