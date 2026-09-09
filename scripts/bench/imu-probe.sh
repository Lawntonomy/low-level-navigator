#!/usr/bin/env bash
# imu-probe.sh -- build, flash and read diagnostics/imu-probe.
#
# Answers system-design research/imu-driver-findings.md §7 on real hardware: is
# an LSM6DSOX on the bus at GP12/GP13, does WHO_AM_I read 0x6C, and does INT1
# on GP14 produce data-ready edges. Does NOT command motion; the probe touches
# GP12/13/14 and nothing else.
#
# Flashing, and why there are two paths:
#   - `picotool load -f` needs the RESIDENT firmware to expose USB stdio, for
#     the vendor reset interface. Any diagnostic probe does; low-level-nav does
#     not (it disables both stdio backends).
#   - So when picotool finds no device, this falls back to enter-bootloader.py,
#     asking the running firmware to hand itself to the bootrom over the
#     command link. Refused while armed -- disarm and re-run.
#   - Both paths failing means the resident image neither exposes USB stdio nor
#     speaks LAWN_ENTER_BOOTLOADER, and that is the one case needing a physical
#     BOOTSEL press.
# Run scripts/bench/link-status.sh first to know which case you are in.
#
# Getting back is easier: imu-probe DOES expose USB stdio, so restore-lln.sh
# needs no bootloader request at all.
#
# Usage:
#   scripts/bench/imu-probe.sh [--seconds N] [--cmd-port PATH] [--cdc PATH]
#
# Env overrides: PI_HOST, PI_HOST_FALLBACK, PI_REMOTE_DIR (lib/common.sh).
#
# shellcheck disable=SC2088  # the ~ below are for the REMOTE shell.
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck source=scripts/bench/lib/common.sh
source "$SCRIPT_DIR/lib/common.sh"
BENCH_RUN="$SCRIPT_DIR/bench-run.sh"

seconds=15
cmd_port=/dev/ttyAMA2
cdc=/dev/ttyACM0
sdk="${PICO_SDK_PATH:-$HOME/Documents/pico-sdk}"

while [[ $# -gt 0 ]]; do
    case "$1" in
        --seconds) seconds="$2"; shift 2 ;;
        --cmd-port) cmd_port="$2"; shift 2 ;;
        --cdc) cdc="$2"; shift 2 ;;
        *) echo "imu-probe: unknown option $1" >&2; exit 64 ;;
    esac
done

repo_root="$(bench_repo_root)"
cd "$repo_root"
host="$(bench_resolve_host)"
probe_dir="$repo_root/diagnostics/imu-probe"
uf2="$probe_dir/build/imu_probe.uf2"

echo "imu-probe: [1/5] build"
"$BENCH_RUN" --local imu-probe-build \
    "cmake -S ${probe_dir} -B ${probe_dir}/build -DPICO_SDK_PATH=${sdk} -DPICO_BOARD=pico2 && make -C ${probe_dir}/build"
[ -f "$uf2" ] || { echo "imu-probe: $uf2 not produced" >&2; exit 1; }

echo "imu-probe: [2/5] copy"
"$BENCH_RUN" --local imu-probe-copy \
    "rsync -a ${uf2} ${host}:~/${PI_REMOTE_DIR}/imu_probe.uf2"

echo "imu-probe: [3/5] flash"
if "$BENCH_RUN" imu-probe-flash \
    "~/picotool/picotool load -f -x -v ~/${PI_REMOTE_DIR}/imu_probe.uf2"; then
    echo "imu-probe: flashed via the vendor reset interface"
else
    echo "imu-probe: picotool found no device; asking the resident firmware to"
    echo "           enter the bootloader over the command link"
    # enter-bootloader.py resolves the dialect at ../mavlink/python relative to
    # itself, so the layout must be mirrored, not flattened.
    "$BENCH_RUN" --local imu-probe-sync-bootloader \
        "rsync -a --relative scripts/enter-bootloader.py mavlink/python ${host}:~/${PI_REMOTE_DIR}/"
    if ! "$BENCH_RUN" imu-probe-enter-bootloader \
        "~/link-stub-pi/.venv/bin/python3 -u ~/${PI_REMOTE_DIR}/scripts/enter-bootloader.py --port ${cmd_port}"; then
        echo "imu-probe: could not reach the bootloader. If the request was REFUSED the" >&2
        echo "           machine is armed -- disarm and re-run. If nothing answered at" >&2
        echo "           all, the resident image speaks neither USB stdio nor" >&2
        echo "           LAWN_ENTER_BOOTLOADER: press BOOTSEL once and re-run." >&2
        exit 1
    fi
    "$BENCH_RUN" imu-probe-wait-bootsel \
        "for i in \$(seq 15); do lsusb | grep -q 2e8a:000f && break; sleep 1; done; lsusb | grep 2e8a"
    "$BENCH_RUN" imu-probe-flash-bootsel \
        "~/picotool/picotool load -x -v ~/${PI_REMOTE_DIR}/imu_probe.uf2"
fi

echo "imu-probe: [4/5] waiting for the CDC device to re-enumerate"
"$BENCH_RUN" imu-probe-wait-cdc \
    "for i in \$(seq 15); do [ -e ${cdc} ] && break; sleep 1; done; ls -l ${cdc}"

# The probe measures first and prints from memory afterwards, on a 5 s loop, so
# a fixed window always catches a whole report rather than half of one.
echo "imu-probe: [5/5] reading ${cdc} for ${seconds}s"
"$BENCH_RUN" imu-probe-read "timeout -s INT ${seconds} cat ${cdc}" || true

cat <<'NOTE'

imu-probe: read the VERDICT line.
  SENSOR ALIVE AND INT1 LIVE   -- pass.
  anything else                -- the stage output says which of bus, part or
                                  wire failed, and stage 4 in particular tells
                                  a silent pin apart from a silent sensor.
Put the real firmware back with:  scripts/bench/restore-lln.sh
NOTE
