#!/usr/bin/env bash

set -euo pipefail

EVENT_DEV="/dev/input/by-id/usb-MINJCODE_MINJCODE_MJ2818A_00000000011C-event-kbd"
MOONRAKER_HOST="localhost"
MOONRAKER_PORT="7125"
SPOOLMAN_PREFIX="web+spoolman:s-"
RETRY_DELAY=2

log() {
    printf '[%s] %s\n' "$(date --iso-8601=seconds)" "$*"
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

    if ! curl --silent --show-error --fail \
        -X POST "http://${MOONRAKER_HOST}:${MOONRAKER_PORT}/printer/gcode/script" \
        -H "Content-Type: application/json" \
        -d "$payload" >/dev/null; then
        log "Failed to post spool id '${spool_id}' to Moonraker"
    fi
}

process_line() {
    local line="$1"

    if [[ -z "$line" ]]; then
        return
    fi

    if [[ "$line" == "$SPOOLMAN_PREFIX"* ]]; then
        log "Spoolman code scanned"
        post_next_spool_id "${line#$SPOOLMAN_PREFIX}"
    elif [[ "$line" == http* ]]; then
        log "URL scanned"
        local spool_id
        spool_id=$(printf '%s' "$line" | awk -F'/' 'NF >= 1 {print $NF}')
        if [[ -n "$spool_id" ]]; then
            post_next_spool_id "$spool_id"
        fi
    else
        log "Unhandled scan payload: $line"
    fi
}

main_loop() {
    local -a keys=( "" "ESC" "1" "2" "3" "4" "5" "6" "7" "8" "9" "0" "-" "=" "BACKSPACE" "TAB"
        "q" "w" "e" "r" "t" "y" "u" "i" "o" "p" "[" "]" "ENTER" "CTRL"
        "a" "s" "d" "f" "g" "h" "j" "k" "l" ";" $'\x27' $'\x60' "LSHIFT" "\\" "z" "x"
        "c" "v" "b" "n" "m" "," "." "/" "RSHIFT" "*" "ALT" "SPACE" )

    declare -A shift_map
    shift_map["1"]="!"
    shift_map["2"]="@"
    shift_map["3"]="#"
    shift_map["4"]="$"
    shift_map["5"]="%"
    shift_map["6"]="^"
    shift_map["7"]="&"
    shift_map["8"]="*"
    shift_map["9"]="("
    shift_map["0"]=")"
    shift_map["-"]="_"
    shift_map["="]="+"
    shift_map["["]="{"
    shift_map["]"]="}"
    shift_map["\\"]="|"
    shift_map[";"]=":"
    shift_map["'"]="\""
    shift_map[","]="<"
    shift_map["."]=">"
    shift_map["/"]="?"

    local buffer=""
    local shift_active=0

    log "Waiting for events from $EVENT_DEV"

    while IFS= read -r line; do
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
                        log "Scanned code: $buffer"
                        process_line "$buffer"
                        buffer=""
                        ;;
                    BACKSPACE)
                        buffer=${buffer%?}
                        ;;
                    TAB|CTRL|ALT|RSHIFT|LSHIFT|"*")
                        # Non printable keys already handled
                        ;;
                    *)
                        if (( shift_active )); then
                            if [[ "$keyname" =~ ^[a-z]$ ]]; then
                                keyname=${keyname^^}
                            elif [[ -n "${shift_map[$keyname]:-}" ]]; then
                                keyname=${shift_map[$keyname]}
                            fi
                        fi
                        buffer+="$keyname"
                        ;;
                esac
            elif [[ "$line" =~ "value 0" ]]; then
                if [[ "$keyname" == "LSHIFT" || "$keyname" == "RSHIFT" ]]; then
                    shift_active=0
                fi
            fi
        fi
    done < <(evtest --grab "$EVENT_DEV" 2>/dev/null)
}

require_command evtest
require_command curl

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
