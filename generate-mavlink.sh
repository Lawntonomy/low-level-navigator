#!/usr/bin/env bash
# Generate MAVLink bindings for the Lawntonomy dialect.
#
# This uses the REAL MAVLink toolchain, not a reimplementation. mavgen.py from
# pymavlink emits the upstream C library verbatim (mavlink/c/*.h - helpers,
# checksum, protocol, types) plus per-dialect message headers. A private
# dialect is MAVLink's sanctioned extension mechanism, the same one ArduPilot
# and PX4 use; nothing in the framing, CRC or parser is ours.
#
# PIN THE GENERATOR. Generated output is committed, so a regeneration under a
# different pymavlink can silently change committed headers - and CRC_EXTRA
# mismatches between the two tiers are exactly what MAVLink versioning is meant
# to catch, not introduce. Install with:
#
#   python3 -m venv .venv && .venv/bin/pip install "pymavlink==$PYMAVLINK_PIN"
#
# The dialect is defined in the system-design repo (IF-0001 embeds it, and
# lawntonomy.xml is extracted from it) so both tiers generate from one source.
# Generated output is committed so the firmware build does not need pymavlink.
#
#   ./generate-mavlink.sh      # uses ../system-design/interfaces/lawntonomy.xml
#   ./generate.sh path/to.xml
set -euo pipefail

# Version the committed output was generated with. Bump deliberately, then
# regenerate and review the diff - do not let it drift silently.
PYMAVLINK_PIN="2.4.49"

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
XML="${1:-$HERE/../system-design/interfaces/lawntonomy.xml}"

[ -f "$XML" ] || { echo "dialect not found: $XML" >&2; exit 1; }

# mavgen resolves <include> relative to the XML's own directory, so stage the
# dialect next to minimal.xml rather than polluting the system-design repo.
PY="${PYTHON:-python3}"
ACTUAL="$("$PY" -c 'import pymavlink; print(pymavlink.__version__)' 2>/dev/null || true)"
if [ -n "$ACTUAL" ] && [ "$ACTUAL" != "$PYMAVLINK_PIN" ]; then
  echo "WARNING: pymavlink $ACTUAL, but committed output was generated with $PYMAVLINK_PIN." >&2
  echo "         Regenerating will produce a diff unrelated to the dialect." >&2
fi

"$PY" -c 'import pymavlink' 2>/dev/null || {
  echo "pymavlink not importable by '$PY'." >&2
  echo "  python3 -m venv .venv && .venv/bin/pip install pymavlink" >&2
  echo "  PYTHON=.venv/bin/python ./generate.sh" >&2
  exit 1
}

STAGE="$(mktemp -d)"
trap 'rm -rf "$STAGE"' EXIT
DEFS="$("$PY" -c 'import pymavlink,os;print(os.path.join(os.path.dirname(pymavlink.__file__),"dialects","v20"))')"

cp "$XML" "$STAGE/lawntonomy.xml"
cp "$DEFS/minimal.xml" "$STAGE/minimal.xml"

# Use the installed console script next to the interpreter. The copy inside
# pymavlink/generator/ uses relative imports and cannot be run as a script.
MAVGEN="$("$PY" - <<'EOF'
import os, shutil, sys
cand = os.path.join(os.path.dirname(sys.executable), "mavgen.py")
print(cand if os.path.exists(cand) else (shutil.which("mavgen.py") or ""))
EOF
)"
[ -n "$MAVGEN" ] && [ -f "$MAVGEN" ] || { echo "mavgen.py not found next to $PY" >&2; exit 1; }

rm -rf "$HERE/mavlink/c" "$HERE/mavlink/python"

echo "== C (Pico) =="
"$PY" "$MAVGEN" --lang=C --wire-protocol=2.0 -o "$HERE/mavlink/c" "$STAGE/lawntonomy.xml"

echo "== Python (Pi) =="
mkdir -p "$HERE/mavlink/python"   # mavgen_python does not create the parent directory
"$PY" "$MAVGEN" --lang=Python --wire-protocol=2.0 \
      -o "$HERE/mavlink/python/lawntonomy.py" "$STAGE/lawntonomy.xml"

echo
echo "Generated from: $XML"
echo "  mavlink/c/lawntonomy/mavlink.h"
echo "  mavlink/python/lawntonomy.py"
