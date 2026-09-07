#!/usr/bin/env bash
# Shared config and safety guards for scripts/bench/*.sh. Sourced, never run
# directly -- every other script under scripts/bench/ sources this so the
# host, remote directory, and the sudo guard live in one place instead of
# being copy-pasted (and drifting) across scripts.
#
# Deliberately does not `set -e`/`-u`/`-o pipefail`: this file is sourced into
# scripts that choose their own options, and setting them here would silently
# change a caller that turned one off on purpose.

: "${PI_HOST:=andrew@raspberrypi.local}"
: "${PI_HOST_FALLBACK:=andrew@192.168.10.230}"
# Both point at the same box; raspberrypi.local (mDNS) is flaky for a minute
# after cold boot, the IP is the fallback. See scripts/bench/README.md.

: "${PI_REMOTE_DIR:=lln-bench}"
# Relative to the andrew home dir on the Pi, i.e. ~/lln-bench. Scripts that
# copy files there must mirror this repo's relative layout underneath it --
# scripts/enter-bootloader.py imports ../mavlink/python relative to itself,
# and that only resolves if mavlink/python/ is copied to the same place
# alongside it.

# bench_resolve_host
#   Prints whichever of PI_HOST / PI_HOST_FALLBACK answers ssh right now, and
#   returns 0. Returns 1 with a message on stderr if neither does.
bench_resolve_host() {
    local host
    for host in "$PI_HOST" "$PI_HOST_FALLBACK"; do
        if ssh -o BatchMode=yes -o ConnectTimeout=5 \
               -o StrictHostKeyChecking=accept-new \
               "$host" true 2>/dev/null; then
            echo "$host"
            return 0
        fi
    done
    echo "bench: cannot reach the Pi as either $PI_HOST or $PI_HOST_FALLBACK" >&2
    return 1
}

# bench_refuse_sudo <command-string>
#   Refuses outright if the command contains "sudo". The bench Pi's sudo needs
#   an interactive password nobody has (see CLAUDE.md / README.md); a script
#   that tries it anyway just hangs at a prompt that will never be answered,
#   which on a machine with motors attached is worse than failing fast.
bench_refuse_sudo() {
    case "$1" in
        *sudo*)
            echo "bench: refusing a command containing 'sudo' -- the bench Pi's" >&2
            echo "       sudo needs an interactive password nobody has." >&2
            echo "       Ask Andrew to run it directly. See scripts/bench/README.md." >&2
            return 1
            ;;
    esac
    return 0
}

# bench_repo_root
#   Prints the low-level-navigator repo root, regardless of the caller's cwd.
bench_repo_root() {
    local here
    here="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
    git -C "$here" rev-parse --show-toplevel 2>/dev/null || (cd "$here/../../.." && pwd)
}
