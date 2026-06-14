#!/bin/sh
# SPDX-License-Identifier: GPL-3.0-or-later
#
# install-remote-screen.sh — wire the U1 Remote Screen (fb-http -> Mainsail) into
# the display's own init script so it starts on EVERY boot.
#
# Why we can't just drop in a new init script:
#   The U1 rootfs is an overlay pivot_root'd by S01aoverlayfs. /etc/init.d/rcS
#   evaluates `for i in /etc/init.d/S??*` ONCE, before that pivot, so rcS only
#   ever runs the BASE-image set of S* scripts. Any S* script we add into the
#   overlay is never in that list and never runs at boot (works by hand, never at
#   boot). Editing a base script DOES run, though: its name is in rcS's captured
#   list and it's exec'd by path after the pivot, so it reads the edited content.
#
# So we inline the launcher directly into the base script S99fb-http (the display
# init the firmware already restarts to toggle the screen). Idempotent (safe to
# re-run, e.g. after an update reverts it): backs up, syntax-checks, marker-guards.
#
# Usage (run on the U1 as root):
#   sh install-remote-screen.sh

set -e

HOOK=/etc/init.d/S99fb-http
MARKER="AFC-REMOTE-SCREEN-HOOK"

if [ ! -f "$HOOK" ]; then
    echo "ERROR: '$HOOK' not found — can't wire the remote screen at boot." >&2
    exit 1
fi

# Remove a previous standalone launcher if it exists (folded in now).
if [ -e /etc/init.d/S98remote-screen ]; then
    /etc/init.d/S98remote-screen stop 2>/dev/null || true
    rm -f /etc/init.d/S98remote-screen
    echo "Removed obsolete /etc/init.d/S98remote-screen"
fi

if grep -q "$MARKER" "$HOOK"; then
    echo "Already wired into $HOOK"
else
    BAK="$HOOK.afcbak.$(date +%s 2>/dev/null || echo bak)"
    cp "$HOOK" "$BAK"
    TMP="$HOOK.tmp.$$"

    # Insert our launcher block before the FIRST standalone `exit` line (the
    # `exit 0` right after helix is handed off), so it runs in the active path.
    LN=$(grep -n -E '^[[:space:]]*exit[[:space:]]' "$HOOK" | head -1 | cut -d: -f1)
    if [ -n "$LN" ]; then
        head -n "$((LN - 1))" "$HOOK" > "$TMP"
    else
        cp "$HOOK" "$TMP"   # no exit to anchor on — append at end
    fi

    cat >> "$TMP" <<'BLOCK'
# AFC-REMOTE-SCREEN-HOOK: serve the DRM display to Mainsail via fb-http.
afc_remote_screen() {
    _bin=/usr/local/bin/fb-http.py
    _pid=/var/run/remote-screen.pid
    [ -f "$_bin" ] || return 0
    case "$1" in
        start)
            _cfg=""
            for _c in /oem/printer_data/config/extended/extended2.cfg \
                      /home/lava/printer_data/config/extended/extended2.cfg; do
                [ -f "$_c" ] && { _cfg="$_c"; break; }
            done
            if [ -n "$_cfg" ] && [ -x /usr/local/bin/extended-config.py ]; then
                _rs=$(/usr/local/bin/extended-config.py get "$_cfg" web remote_screen false 2>/dev/null)
                case "$_rs" in [Tt][Rr][Uu][Ee]) ;; *) return 0 ;; esac
            fi
            [ -f "$_pid" ] && kill -0 "$(cat "$_pid")" 2>/dev/null && return 0
            start-stop-daemon -S -b -m -p "$_pid" -x /bin/sh -- -c \
                "exec /usr/bin/python3 $_bin --bind 127.0.0.1 --port 8092 --backend drm --drm-device /dev/dri/card0 --drm-wait 60 --html-dir /usr/local/share/fb-http/html >/tmp/fb-http.log 2>&1"
            ;;
        stop)
            if [ -f "$_pid" ]; then
                start-stop-daemon -K -p "$_pid" -s TERM; rm -f "$_pid"
            else
                pkill -f "$_bin" 2>/dev/null
            fi
            ;;
    esac
}
afc_remote_screen "$1"
BLOCK

    if [ -n "$LN" ]; then
        tail -n +"$LN" "$HOOK" >> "$TMP"
    fi

    if sh -n "$TMP" 2>/dev/null; then
        mv "$TMP" "$HOOK"
        chmod +x "$HOOK" 2>/dev/null || true
        echo "Wired remote screen into: $HOOK"
        echo "Backup: $BAK"
    else
        rm -f "$TMP"
        echo "ERROR: patched '$HOOK' failed syntax check; left it unchanged." >&2
        echo "       Backup is at '$BAK'." >&2
        exit 1
    fi
fi

echo "Restarting the screen..."
"$HOOK" restart || true
echo "Done. Reboot to confirm it comes up on its own."
