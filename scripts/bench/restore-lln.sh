#!/usr/bin/env bash
# restore-lln.sh -- put the main firmware back after a diagnostic probe.
#
# Why this exists next to cal0-check.sh, which also flashes low-level-nav:
# cal0-check.sh reaches the bootloader by asking the RESIDENT firmware to enter
# it over the command link, which only works if that firmware is already a
# low-level-nav build. After a probe run it is not -- and it does not need to
# be, because every diagnostic probe exposes USB stdio, so `picotool load -f`
# has a vendor reset interface and works directly. This script is that
# direction: probe -> low-level-nav, with no BOOTSEL press and no command-link
# request. Once it has run, cal0-check.sh is the way to reflash again.
#
# PRECONDITION, procedural because nothing in the firmware checks it: have the
# wheels off the ground. This flashes with -x, so the firmware starts executing
# the moment the write finishes, and the TB6612 breakout's R1 pull-up holds
# STBY high -- the driver is ENABLED from power-on until firmware drives GPIO 2
# low (SAF-19, ADR-0010). This script commands no motion itself, and as of this
# writing control_task calls motors::safe_state() unconditionally, but that is
# a fact about the current firmware, not a guarantee from this script.
#
# Usage:
#   scripts/bench/restore-lln.sh [--uf2 PATH]
#
# Env overrides: PI_HOST, PI_HOST_FALLBACK, PI_REMOTE_DIR (lib/common.sh).
#
# shellcheck disable=SC2088  # the ~ below are for the REMOTE shell.
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck source=scripts/bench/lib/common.sh
source "$SCRIPT_DIR/lib/common.sh"
BENCH_RUN="$SCRIPT_DIR/bench-run.sh"

repo_root="$(bench_repo_root)"
cd "$repo_root"
uf2="$repo_root/build/low-level-nav.uf2"
cmd_port="${CMD_PORT:-/dev/ttyAMA2}"

while [[ $# -gt 0 ]]; do
    case "$1" in
        --uf2) uf2="$2"; shift 2 ;;
        *) echo "restore-lln: unknown option $1" >&2; exit 64 ;;
    esac
done

[ -f "$uf2" ] || {
    echo "restore-lln: $uf2 does not exist. Build it first:" >&2
    echo "  docker run --rm -v \$(pwd):/workspace low-level-navigator" >&2
    exit 1
}

host="$(bench_resolve_host)"

echo "restore-lln: PRECONDITION -- wheels off the ground. Nothing here verifies that."
echo "restore-lln: [1/2] copy"
"$BENCH_RUN" --local restore-lln-copy \
    "rsync -a ${uf2} ${host}:~/${PI_REMOTE_DIR}/low-level-nav.uf2"

echo "restore-lln: [2/2] flash"
if ! "$BENCH_RUN" restore-lln-flash \
    "~/picotool/picotool load -f -x -v ~/${PI_REMOTE_DIR}/low-level-nav.uf2"; then
    # The fallback this script was missing. Without it, a lln -> lln reflash
    # failed with a message telling the reader to do by hand what the other two
    # bench scripts already do automatically -- and on 2026-09-10 that silently
    # cost a console capture, because the board never rebooted and the empty log
    # looked like a wiring fault.
    echo "restore-lln: picotool found no device; asking the resident firmware to"
    echo "             enter the bootloader over the command link"
    "$BENCH_RUN" --local restore-lln-sync-bootloader \
        "rsync -a --relative scripts/enter-bootloader.py mavlink/python ${host}:~/${PI_REMOTE_DIR}/"
    if ! "$BENCH_RUN" restore-lln-enter-bootloader \
        "~/link-stub-pi/.venv/bin/python3 -u ~/${PI_REMOTE_DIR}/scripts/enter-bootloader.py --port ${cmd_port}"; then
        echo "restore-lln: could not reach the bootloader. REFUSED means armed --" >&2
        echo "             disarm and re-run. Silence means the resident image speaks" >&2
        echo "             neither USB stdio nor LAWN_ENTER_BOOTLOADER: press BOOTSEL." >&2
        exit 1
    fi
    "$BENCH_RUN" restore-lln-wait-bootsel \
        "for i in \$(seq 15); do lsusb | grep -q 2e8a:000f && break; sleep 1; done; lsusb | grep 2e8a" ||
        { echo "restore-lln: BOOTSEL never enumerated. Press BOOTSEL and re-run." >&2; exit 1; }
    "$BENCH_RUN" restore-lln-flash-bootsel \
        "~/picotool/picotool load -x -v ~/${PI_REMOTE_DIR}/low-level-nav.uf2" ||
        { echo "restore-lln: flash failed after reaching BOOTSEL." >&2; exit 1; }
fi

cat <<'NOTE'

restore-lln: low-level-nav is resident again. It has no USB stdio, so reflashing
from here goes through the command link (cal0-check.sh, or
scripts/enter-bootloader.py). Confirm it is alive with:
  scripts/bench/link-status.sh
NOTE
