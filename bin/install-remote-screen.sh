#!/bin/sh
# SPDX-License-Identifier: GPL-3.0-or-later
#
# install-remote-screen.sh — install the U1 Remote Screen (fb-http -> Mainsail)
# so it starts on EVERY boot on this locked-down overlay rootfs.
#
# Why a plain init script isn't enough on the U1:
#   /etc/init.d/rcS evaluates `for i in /etc/init.d/S??*` ONCE, at loop start,
#   BEFORE S01aoverlayfs pivot_root's the persistent overlay (/oem/overlay/upper)
#   into /. So rcS only ever runs the BASE-image set of S* scripts; any S* script
#   we add into the overlay is never in that list and never runs at boot (it works
#   when started by hand, never at boot — exactly the symptom we hit).
#
#   An EDIT to an existing base script DOES run, though: its name is already in
#   rcS's captured list, and rcS exec's it by path AFTER the pivot, so it reads
#   the overlay-merged (edited) content. So we ride a stable base script.
#
# This installer is IDEMPOTENT (safe to re-run, e.g. after an update reverts the
# hook). It:
#   1. installs /etc/init.d/S98remote-screen (the actual launcher),
#   2. appends a marker-guarded hook to the base script S99openrfid that calls it,
#   3. starts it now.
#
# Usage (run on the U1 as root):
#   sh install-remote-screen.sh
#   sh install-remote-screen.sh /path/to/S98remote-screen   # explicit launcher src

set -e

SELF_DIR=$(dirname "$0")
LAUNCHER_SRC="${1:-$SELF_DIR/S98remote-screen}"
LAUNCHER_DST=/etc/init.d/S98remote-screen
# Hook the display's own base init script (helix-managed but user-owned). It runs
# at boot and exit 0's right after delegating to helix; we insert our call before
# that first exit so the screen starts once helix has been handed off.
HOOK=/etc/init.d/S99fb-http
MARKER="AFC-REMOTE-SCREEN-HOOK"

# 1. Install the launcher.
if [ ! -f "$LAUNCHER_SRC" ]; then
    echo "ERROR: launcher source '$LAUNCHER_SRC' not found." >&2
    echo "       Pass the path to S98remote-screen as the first argument." >&2
    exit 1
fi
cp "$LAUNCHER_SRC" "$LAUNCHER_DST"
chmod +x "$LAUNCHER_DST"
echo "Installed launcher: $LAUNCHER_DST"

# 2. Hook a base script that actually runs at boot.
if [ ! -f "$HOOK" ]; then
    echo "ERROR: base hook script '$HOOK' not found — can't wire boot start." >&2
    exit 1
fi

if grep -q "$MARKER" "$HOOK"; then
    echo "Hook already present in $HOOK"
else
    BAK="$HOOK.afcbak.$(date +%s 2>/dev/null || echo bak)"
    cp "$HOOK" "$BAK"
    TMP="$HOOK.tmp.$$"
    # Insert our start/stop delegation before the FIRST standalone `exit` line
    # (matches `exit 0` in S99fb-http and `exit $?` elsewhere). The done flag
    # keeps it to the first match so it lands in the active code path, before the
    # script exits. Guarded by the marker.
    awk -v done=0 '
        /^[[:space:]]*exit[[:space:]]/ && !done {
            print "# " "AFC-REMOTE-SCREEN-HOOK: start the Mainsail remote screen"
            print "case \"$1\" in"
            print "  start) [ -x /etc/init.d/S98remote-screen ] && /etc/init.d/S98remote-screen start ;;"
            print "  stop)  [ -x /etc/init.d/S98remote-screen ] && /etc/init.d/S98remote-screen stop ;;"
            print "esac"
            done=1
        }
        { print }
    ' "$HOOK" > "$TMP"

    if ! grep -q "$MARKER" "$TMP"; then
        # No `exit $?` to anchor to — append at end of file instead.
        {
            echo ""
            echo "# $MARKER: start the Mainsail remote screen"
            echo 'case "$1" in'
            echo '  start) [ -x /etc/init.d/S98remote-screen ] && /etc/init.d/S98remote-screen start ;;'
            echo '  stop)  [ -x /etc/init.d/S98remote-screen ] && /etc/init.d/S98remote-screen stop ;;'
            echo 'esac'
        } >> "$TMP"
    fi

    if sh -n "$TMP" 2>/dev/null; then
        mv "$TMP" "$HOOK"
        chmod +x "$HOOK" 2>/dev/null || true
        echo "Hooked boot start into: $HOOK"
        echo "Backup: $BAK"
    else
        rm -f "$TMP"
        echo "ERROR: patched '$HOOK' failed syntax check; left it unchanged." >&2
        echo "       Backup is at '$BAK'." >&2
        exit 1
    fi
fi

# 3. Start it now.
echo "Starting now..."
"$LAUNCHER_DST" restart || true
sleep 2
"$LAUNCHER_DST" status || true
echo "Done. Reboot to confirm it comes up on its own."
