#!/usr/bin/env bash
# flash-diag.sh -- build, flash and read any diagnostic under diagnostics/.
#
# Generalised out of imu-probe.sh rather than copied from it: the build/copy/
# flash/read sequence and the two-path flashing logic are identical for every
# diagnostic, and a second copy would drift. imu-probe.sh keeps its own script
# because it carries probe-specific guidance about reading the verdict.
#
# Flashing takes one of two paths, which is the whole awkwardness:
#   - `picotool load -f` needs the RESIDENT firmware to expose USB stdio. Every
#     diagnostic does; low-level-nav does not.
#   - So when picotool finds no device, this asks the running firmware to enter
#     the bootloader over the command link. Refused while armed -- disarm first.
#   - Both failing means a physical BOOTSEL press.
#
# Usage:
#   scripts/bench/flash-diag.sh <dir-under-diagnostics> <cmake-target> [--seconds N]
#   scripts/bench/flash-diag.sh imu-init imu_init --seconds 12
#
# Env overrides: PI_HOST, PI_HOST_FALLBACK, PI_REMOTE_DIR (lib/common.sh).
#
# shellcheck disable=SC2088  # the ~ below are for the REMOTE shell.
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck source=scripts/bench/lib/common.sh
source "$SCRIPT_DIR/lib/common.sh"
BENCH_RUN="$SCRIPT_DIR/bench-run.sh"

[[ $# -ge 2 ]] || { echo "usage: flash-diag.sh <dir> <target> [--seconds N]" >&2; exit 64; }
diag="$1"; target="$2"; shift 2
seconds=12
cdc=/dev/ttyACM0
cmd_port=/dev/ttyAMA2
sdk="${PICO_SDK_PATH:-$HOME/Documents/pico-sdk}"
while [[ $# -gt 0 ]]; do
    case "$1" in
        --seconds) seconds="$2"; shift 2 ;;
        --cdc) cdc="$2"; shift 2 ;;
        *) echo "flash-diag: unknown option $1" >&2; exit 64 ;;
    esac
done

repo_root="$(bench_repo_root)"
cd "$repo_root"
host="$(bench_resolve_host)"
dir="$repo_root/diagnostics/$diag"
uf2="$dir/build/${target}.uf2"
[ -d "$dir" ] || { echo "flash-diag: no such diagnostic: $dir" >&2; exit 1; }

echo "flash-diag: [1/5] build $diag"
"$BENCH_RUN" --local "$diag-build" \
    "cmake -S ${dir} -B ${dir}/build -DPICO_SDK_PATH=${sdk} -DPICO_BOARD=pico2 && make -C ${dir}/build"
[ -f "$uf2" ] || { echo "flash-diag: $uf2 not produced" >&2; exit 1; }

echo "flash-diag: [2/5] copy"
"$BENCH_RUN" --local "$diag-copy" "rsync -a ${uf2} ${host}:~/${PI_REMOTE_DIR}/${target}.uf2"

echo "flash-diag: [3/5] flash"
if "$BENCH_RUN" "$diag-flash" "~/picotool/picotool load -f -x -v ~/${PI_REMOTE_DIR}/${target}.uf2"; then
    echo "flash-diag: flashed via the vendor reset interface"
else
    echo "flash-diag: picotool found no device; asking the resident firmware to"
    echo "            enter the bootloader over the command link"
    "$BENCH_RUN" --local "$diag-sync-bootloader" \
        "rsync -a --relative scripts/enter-bootloader.py mavlink/python ${host}:~/${PI_REMOTE_DIR}/"
    if ! "$BENCH_RUN" "$diag-enter-bootloader" \
        "~/link-stub-pi/.venv/bin/python3 -u ~/${PI_REMOTE_DIR}/scripts/enter-bootloader.py --port ${cmd_port}"; then
        echo "flash-diag: could not reach the bootloader. REFUSED means the machine is" >&2
        echo "            armed -- disarm and re-run. Silence means the resident image" >&2
        echo "            speaks neither USB stdio nor LAWN_ENTER_BOOTLOADER: press BOOTSEL." >&2
        exit 1
    fi
    "$BENCH_RUN" "$diag-wait-bootsel" \
        "for i in \$(seq 15); do lsusb | grep -q 2e8a:000f && break; sleep 1; done; lsusb | grep 2e8a"
    "$BENCH_RUN" "$diag-flash-bootsel" "~/picotool/picotool load -x -v ~/${PI_REMOTE_DIR}/${target}.uf2"
fi

echo "flash-diag: [4/5] waiting for the CDC device"
"$BENCH_RUN" "$diag-wait-cdc" \
    "for i in \$(seq 15); do [ -e ${cdc} ] && break; sleep 1; done; ls -l ${cdc}"

echo "flash-diag: [5/5] reading ${cdc} for ${seconds}s"
"$BENCH_RUN" "$diag-read" "timeout -s INT ${seconds} cat ${cdc}" || true

echo
echo "flash-diag: put the real firmware back with scripts/bench/restore-lln.sh"
