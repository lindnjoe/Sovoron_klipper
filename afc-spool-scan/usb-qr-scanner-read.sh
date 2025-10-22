#!/usr/bin/env bash

set -euo pipefail

EVENT_DEV="/dev/input/by-id/usb-MINJCODE_MINJCODE_MJ2818A_00000000011C-event-kbd"
MOONRAKER_HOST="localhost"
MOONRAKER_PORT="7125"
SPOOLMAN_PREFIX="web+spoolman:s-"
RETRY_DELAY=2
EVTEST_GRAB_SUPPORTED=""

log() {
    printf '[%s] %s\n' "$(date --iso-8601=seconds)" "$*" >&2
}

require_command() {
    local cmd="$1"
    if ! command -v "$cmd" >/dev/null 2>&1; then
        log "Error: required command '$cmd' not found in PATH"
        exit 1
    fi
}

post_next_spool_id() {
    local spool_id="$1"
    local payload
    payload=$(printf '{"script": "SET_NEXT_SPOOL_ID SPOOL_ID=%s"}' "$spool_id")

    if curl --silent --show-error --fail \
        -X POST "http://${MOONRAKER_HOST}:${MOONRAKER_PORT}/printer/gcode/script" \
        -H "Content-Type: application/json" \
        -d "$payload" >/dev/null; then
        log "Posted spool id '${spool_id}' to Moonraker"
    else
        log "Failed to post spool id '${spool_id}' to Moonraker"
    fi
}

process_line() {
    local line="$1"

    if [[ -z "$line" ]]; then
        return
    fi

    line="${line//$'\r'/}"

    if [[ "$line" == "$SPOOLMAN_PREFIX"* ]]; then
        local spool_id="${line#$SPOOLMAN_PREFIX}"
        log "Spoolman code scanned: $spool_id"
        post_next_spool_id "$spool_id"
    elif [[ "$line" =~ ^https?:// ]]; then
        local spool_id=""
        if ! spool_id=$(printf '%s' "$line" | grep -oE '[^/#?]+$'); then
            spool_id=""
        fi
        if [[ -n "$spool_id" ]]; then
            log "URL scanned, extracted spool id: $spool_id"
            post_next_spool_id "$spool_id"
        else
            log "URL scanned but unable to extract spool id: $line"
        fi
    else
        log "Unhandled scan payload: $line"
    fi
}

translate_shifted() {
    local key="$1"
    case "$key" in
        [a-z]) printf '%s' "${key^^}"; return 0 ;;
        1) printf '!'; return 0 ;;
        2) printf '@'; return 0 ;;
        3) printf '#'; return 0 ;;
        4) printf '$'; return 0 ;;
        5) printf '%%'; return 0 ;;
        6) printf '^'; return 0 ;;
        7) printf '&'; return 0 ;;
        8) printf '*'; return 0 ;;
        9) printf '('; return 0 ;;
        0) printf ')'; return 0 ;;
        -) printf '_'; return 0 ;;
        =) printf '+'; return 0 ;;
        '[') printf '{'; return 0 ;;
        ']') printf '}'; return 0 ;;
        '\\') printf '|'; return 0 ;;
        ';') printf ':'; return 0 ;;
        "'") printf '"'; return 0 ;;
        ,) printf '<'; return 0 ;;
        .) printf '>'; return 0 ;;
        /) printf '?'; return 0 ;;
        '`') printf '~'; return 0 ;;
        *) return 1 ;;
    esac
}

evtest_supports_grab() {
    if [[ -n "$EVTEST_GRAB_SUPPORTED" ]]; then
        [[ "$EVTEST_GRAB_SUPPORTED" == "yes" ]]
        return
    fi

    if evtest --help 2>&1 | grep -q -- '--grab'; then
        EVTEST_GRAB_SUPPORTED="yes"
        log "evtest supports --grab; running in exclusive mode"
        return 0
    fi

    EVTEST_GRAB_SUPPORTED="no"
    log "evtest does not support --grab; continuing without exclusive mode"
    return 1
}

start_evtest_stream() {
    local -a cmd
    if command -v stdbuf >/dev/null 2>&1; then
        cmd=(stdbuf -oL -eL evtest)
    else
        cmd=(evtest)
    fi

    if evtest_supports_grab; then
        cmd+=(--grab)
    fi

    cmd+=("$EVENT_DEV")

    log "Starting ${cmd[*]}"
    "${cmd[@]}"
}

main_loop() {
    local -a keys=( "" "ESC" "1" "2" "3" "4" "5" "6" "7" "8" "9" "0" "-" "=" "BACKSPACE" "TAB" \
        "q" "w" "e" "r" "t" "y" "u" "i" "o" "p" "[" "]" "ENTER" "CTRL" \
        "a" "s" "d" "f" "g" "h" "j" "k" "l" ";" "'" "\`" "LSHIFT" "\\" "z" "x" \
        "c" "v" "b" "n" "m" "," "." "/" "RSHIFT" "*" "ALT" "SPACE" )

    local buffer=""
    local shift_active=0

    log "Waiting for events from $EVENT_DEV"

    start_evtest_stream | while IFS= read -r line; do
        [[ -z "$line" ]] && continue

        if [[ "$line" =~ EV_KEY ]]; then
            local keycode
            keycode=$(sed -n 's/.*code \([0-9]\+\) (.*/\1/p' <<<"$line")
            [[ -z "$keycode" ]] && continue

            local keyname="${keys[$keycode]:-}"
            [[ -z "$keyname" ]] && continue

            if [[ "$line" =~ "value 1" ]]; then
                case "$keyname" in
                    LSHIFT|RSHIFT)
                        shift_active=1
                        ;;
                    ENTER)
                        local trimmed_buffer="${buffer//$'\r'/}"
                        log "Scanned code: $trimmed_buffer"
                        process_line "$trimmed_buffer"
                        buffer=""
                        ;;
                    BACKSPACE)
                        buffer=${buffer%?}
                        ;;
                    TAB|CTRL|ALT|RSHIFT|LSHIFT|"*")
                        ;;
                    *)
                        local char="$keyname"
                        if (( shift_active )); then
                            local shifted
                            if shifted=$(translate_shifted "$keyname"); then
                                char="$shifted"
                            fi
                        fi
                        buffer+="$char"
                        if (( shift_active )); then
                            # Some scanners never emit a shift release; reset to avoid stuck uppercase
                            shift_active=0
                        fi
                        ;;
                esac
            elif [[ "$line" =~ "value 0" ]]; then
                if [[ "$keyname" == "LSHIFT" || "$keyname" == "RSHIFT" ]]; then
                    shift_active=0
                fi
            elif [[ "$line" =~ "value 2" ]]; then
                continue
            fi
        fi
    done
}

require_command evtest
require_command curl
require_command grep

while true; do
    if [[ ! -e "$EVENT_DEV" ]]; then
        log "Device $EVENT_DEV not found. Waiting..."
        sleep "$RETRY_DELAY"
        continue
    fi

    if ! main_loop; then
        log "Event loop exited unexpectedly, retrying in ${RETRY_DELAY}s"
        sleep "$RETRY_DELAY"
    fi

done
