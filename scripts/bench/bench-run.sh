#!/usr/bin/env bash
# bench-run.sh -- run one command against the bench hardware, logged.
#
# This is the wrapper every other script under scripts/bench/ uses, so that no
# command reaching the Pi or the Pico is an ad-hoc `ssh andrew@...` one-liner
# composed in the moment. It runs the given command (by default over ssh on
# the Pi; with --local, on this machine instead), and appends to
# bench-logs/<UTC timestamp>-<label>.log: the exact command, the target, the
# start time, full stdout+stderr interleaved, the exit status, and the
# duration. Everything is echoed to the terminal too. Exit status propagates.
#
# Usage:
#   scripts/bench/bench-run.sh [--local] <label> <command...>
#
# <command...> is joined with single spaces and handed to a shell (the
# remote login shell over ssh, or `bash -c` locally) as ONE command line, so
# pipes/redirection/quoting inside it work the way they would if you had
# typed them straight after `ssh host`. If the command needs its own internal
# quoting, quote the whole thing as one argument.
#
# --local runs the command on THIS machine instead of over ssh. Use it for
# steps that originate here and reach out themselves -- rsync/scp pushing
# files to the Pi -- while still getting the same logging. Nothing that needs
# to run on the Pi (picotool, anything touching /dev/tty*, enter-bootloader.py)
# is a --local step, because those devices only exist on the Pi.
#
# label becomes part of the log filename; keep it to [A-Za-z0-9_-].
#
# Env overrides: PI_HOST, PI_HOST_FALLBACK, PI_REMOTE_DIR (scripts/bench/lib/common.sh).
#
# Examples:
#   scripts/bench/bench-run.sh picotool-info '~/picotool/picotool info'
#   scripts/bench/bench-run.sh --local sync-scripts \
#       'rsync -a --relative scripts/enter-bootloader.py mavlink/python andrew@raspberrypi.local:~/lln-bench/'
set -uo pipefail  # not -e: the command's exit status is the whole point, not a reason to bail early

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck source=scripts/bench/lib/common.sh
source "$SCRIPT_DIR/lib/common.sh"

usage() {
    cat <<'EOF'
Usage: scripts/bench/bench-run.sh [--local] <label> <command...>

Runs <command...> (joined with spaces, as one shell command line) on the
bench Pi over ssh, or on this machine with --local, logging the exact
command, start time, full interleaved stdout+stderr, exit status and
duration to bench-logs/<UTC timestamp>-<label>.log. Exit status propagates.

See the header comment in this file, and scripts/bench/README.md, for the
full explanation and examples.
EOF
}

local_mode=0
if [[ "${1:-}" == "--local" ]]; then
    local_mode=1
    shift
fi

if [[ $# -lt 2 ]]; then
    usage >&2
    exit 64
fi

label="$1"
shift
cmd="$*"

if [[ ! "$label" =~ ^[A-Za-z0-9_-]+$ ]]; then
    echo "bench-run: label '$label' must match [A-Za-z0-9_-]+" >&2
    exit 64
fi

bench_refuse_sudo "$cmd" || exit 1

repo_root="$(bench_repo_root)"
log_dir="$repo_root/bench-logs"
mkdir -p "$log_dir"

timestamp="$(date -u +%Y%m%dT%H%M%SZ)"
log="$log_dir/${timestamp}-${label}.log"

if [[ $local_mode -eq 1 ]]; then
    target_desc="local ($(hostname))"
else
    host="$(bench_resolve_host)" || exit 1
    target_desc="ssh $host"
fi

{
    echo "=== bench-run ==="
    echo "label:    $label"
    echo "target:   $target_desc"
    echo "command:  $cmd"
    echo "start:    $(date -u +%Y-%m-%dT%H:%M:%SZ)"
    echo "---- output (stdout+stderr interleaved) ----"
} | tee -a "$log"

start_ts=$(date +%s)
if [[ $local_mode -eq 1 ]]; then
    bash -c "$cmd" 2>&1 | tee -a "$log"
    status=${PIPESTATUS[0]}
else
    # ConnectTimeout only bounds the TCP/auth handshake, not the command
    # itself -- a long-running capture on the far side is expected to run for
    # as long as it runs.
    ssh -o ConnectTimeout=10 "$host" "$cmd" 2>&1 | tee -a "$log"
    status=${PIPESTATUS[0]}
fi
end_ts=$(date +%s)

{
    echo "---- end output ----"
    echo "exit:     $status"
    echo "duration: $((end_ts - start_ts))s"
    echo "end:      $(date -u +%Y-%m-%dT%H:%M:%SZ)"
} | tee -a "$log"

echo "bench-run: log: $log" >&2
exit "$status"
