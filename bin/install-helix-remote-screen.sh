#!/bin/sh
# SPDX-License-Identifier: GPL-3.0-or-later
#
# install-helix-remote-screen.sh
#
# Wires the Snapmaker U1 "Remote Screen" (fb-http -> Mainsail) into HelixScreen's
# boot path. HelixScreen's installer hijacks /etc/init.d/S99fb-http to launch
# its own UI and never starts fb-http, so the stock Remote Screen feature stops
# working. This patches HelixScreen's platform hook (platform_pre_start) to
# launch /usr/local/bin/fb-http.py --backend drm when Remote Screen is enabled.
#
# It is IDEMPOTENT (safe to re-run, e.g. after a HelixScreen update overwrites
# the hook file), backs the hook file up, validates shell syntax before saving,
# and only touches the file via an atomic temp+rename.
#
# Usage (run on the U1 as root):
#   sh install-helix-remote-screen.sh                # default hook path
#   sh install-helix-remote-screen.sh /path/to/hooks.sh
#
# After running: copy the latest bin/fb-http.py to /usr/local/bin/fb-http.py,
# then `/etc/init.d/S99fb-http restart`.

set -e

HOOKS="${1:-/userdata/helixscreen/platform/hooks.sh}"
[ -f "$HOOKS" ] || HOOKS="/opt/helixscreen/platform/hooks.sh"

MARKER="AFC-REMOTE-SCREEN-HOOK"

if [ ! -f "$HOOKS" ]; then
    echo "ERROR: HelixScreen platform hooks not found at '$HOOKS'." >&2
    echo "       Is HelixScreen installed? Pass the path explicitly if needed." >&2
    exit 1
fi

if grep -q "$MARKER" "$HOOKS"; then
    echo "Already patched: $HOOKS"
    exit 0
fi

if ! grep -q '^platform_pre_start()' "$HOOKS"; then
    echo "ERROR: no platform_pre_start() in '$HOOKS' — unexpected hook layout." >&2
    echo "       Not patching. Send the file to the maintainer." >&2
    exit 1
fi

BAK="$HOOKS.afcbak.$(date +%s 2>/dev/null || echo bak)"
cp "$HOOKS" "$BAK"
TMP="$HOOKS.tmp.$$"

# 1. Insert the ensure_remote_screen call before the `return 0` inside
#    platform_pre_start (preserving everything else in that function).
awk '
    /^platform_pre_start\(\)/ { inpre = 1 }
    inpre && /return 0/ {
        print "    ensure_remote_screen   # " "AFC-REMOTE-SCREEN-HOOK"
        inpre = 0
    }
    { print }
' "$HOOKS" > "$TMP"

# 2. Append the function definition (resolved at call time, so defining it
#    after platform_pre_start is fine).
cat >> "$TMP" <<'EOF'

# AFC-REMOTE-SCREEN-HOOK: serve the DRM display to Mainsail via fb-http.
# HelixScreen's installer hijacked S99fb-http to launch helix, so nothing else
# starts fb-http. Start it (detached) when the Remote Screen setting is enabled;
# fb-http reads the same DRM scanout (card0) helix renders to and waits for an
# active CRTC (--drm-wait), so launching before helix renders is fine.
HELIX_REMOTE_CFG="${HELIX_REMOTE_CFG:-/home/lava/printer_data/config/extended/extended2.cfg}"
ensure_remote_screen() {
    [ -x /usr/local/bin/fb-http.py ] || return 0
    if [ -x /usr/local/bin/extended-config.py ]; then
        _rs=$(/usr/local/bin/extended-config.py get "$HELIX_REMOTE_CFG" \
              web remote_screen false 2>/dev/null)
        case "$_rs" in [Tt][Rr][Uu][Ee]) ;; *) return 0 ;; esac
    fi
    pgrep -f 'fb-http.py' >/dev/null 2>&1 && return 0
    echo "Remote Screen: starting fb-http on :8092 (DRM backend)"
    /usr/bin/python3 /usr/local/bin/fb-http.py \
        --bind 127.0.0.1 --port 8092 --backend drm \
        --html-dir /usr/share/fb-http/html >/tmp/fb-http.log 2>&1 &
}
EOF

# 3. Validate before replacing — never leave a broken hook (helix won't boot).
if sh -n "$TMP" 2>/dev/null; then
    mv "$TMP" "$HOOKS"
    chmod +x "$HOOKS" 2>/dev/null || true
    echo "Patched: $HOOKS"
    echo "Backup:  $BAK"
    echo "Next: deploy bin/fb-http.py to /usr/local/bin/fb-http.py, then"
    echo "      /etc/init.d/S99fb-http restart"
else
    rm -f "$TMP"
    echo "ERROR: generated hook failed 'sh -n' syntax check; left '$HOOKS' unchanged." >&2
    echo "       Backup is at '$BAK'." >&2
    exit 1
fi
