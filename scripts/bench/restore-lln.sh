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
    echo "restore-lln: picotool could not reach the board. If a no-stdio image is" >&2
    echo "             already resident, -f has nothing to talk to: use" >&2
    echo "             scripts/enter-bootloader.py, or press BOOTSEL once." >&2
    exit 1
fi

cat <<'NOTE'

restore-lln: low-level-nav is resident again. It has no USB stdio, so reflashing
from here goes through the command link (cal0-check.sh, or
scripts/enter-bootloader.py). Confirm it is alive with:
  scripts/bench/link-status.sh
NOTE
