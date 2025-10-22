#!/bin/bash

EVENT_DEV="/dev/input/by-id/usb-MINJCODE_MINJCODE_MJ2818A_00000000011C-event-kbd"
MOONRAKER_HOST="localhost"
MOONRAKER_PORT="7125"
SPOOLMAN_PREFIX="web+spoolman:s-"
RETRY_DELAY=2

if ! command -v evtest >/dev/null 2>&1; then
    echo "Error: evtest is not installed or not in PATH."
    exit 1
fi

if ! command -v curl >/dev/null 2>&1; then
    echo "Error: curl is not installed or not in PATH."
    exit 1
fi

post_next_spool_id() {
    local spool_id="$1"
    curl -X POST "http://${MOONRAKER_HOST}:${MOONRAKER_PORT}/printer/gcode/script" \
        -H "Content-Type: application/json" \
        -d "{\"script\": \"SET_NEXT_SPOOL_ID SPOOL_ID=${spool_id}\"}" \
        || echo "Failed to post spool id '${spool_id}'"
}

process_line() {
    local line="$1"

    if [[ -z "$line" ]]; then
        return
    fi

    line="${line//$'\r'/}"

    if [[ "$line" == "$SPOOLMAN_PREFIX"* ]]; then
        echo "Magic code Scanned"
        local spool_id="${line#$SPOOLMAN_PREFIX}"
        post_next_spool_id "$spool_id"
    elif [[ "$line" == http* ]]; then
        echo "URL Scanned"
        local spool_id
        spool_id=$(echo "$line" | cut -d'/' -f6)
        [[ -n "$spool_id" ]] && post_next_spool_id "$spool_id"
    else
        echo "Unhandled scan payload: $line"
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
        "\\") printf '|'; return 0 ;;
        ";") printf ':'; return 0 ;;
        "'") printf '%s' "\""; return 0 ;;
        ,) printf '<'; return 0 ;;
        .) printf '>'; return 0 ;;
        /) printf '?'; return 0 ;;
        '`') printf '~'; return 0 ;;
        *) return 1 ;;
    esac
}

run_event_loop() {
    local keys=( "" "ESC" "1" "2" "3" "4" "5" "6" "7" "8" "9" "0" "-" "=" "BACKSPACE" "TAB" \
        "q" "w" "e" "r" "t" "y" "u" "i" "o" "p" "[" "]" "ENTER" "CTRL" \
        "a" "s" "d" "f" "g" "h" "j" "k" "l" ";" "'" "\`" "LSHIFT" "\\" "z" "x" \
        "c" "v" "b" "n" "m" "," "." "/" "RSHIFT" "*" "ALT" "SPACE" )

    local buffer=""
    local shift_active=0

    local evtest_cmd=(evtest "$EVENT_DEV")
    if command -v stdbuf >/dev/null 2>&1; then
        evtest_cmd=(stdbuf -oL -eL "${evtest_cmd[@]}")
    fi

    "${evtest_cmd[@]}" 2>/dev/null | while read -r line; do
        [[ "$line" == *"EV_KEY"* ]] || continue

        local keycode
        keycode=$(echo "$line" | sed -n 's/.*code \([0-9]\+\) (.*/\1/p')
        [[ -z "$keycode" ]] && continue

        local keyname="${keys[$keycode]}"
        [[ -z "$keyname" ]] && continue

        if [[ "$line" == *"value 1"* ]]; then
            case "$keyname" in
                LSHIFT|RSHIFT)
                    shift_active=1
                    ;;
                ENTER)
                    echo "Scanned code: $buffer"
                    process_line "$buffer"
                    buffer=""
                    shift_active=0
                    ;;
                BACKSPACE)
                    buffer=${buffer%?}
                    ;;
                TAB|CTRL|ALT|"*")
                    ;;
                *)
                    if [[ $shift_active -eq 1 ]]; then
                        local shifted
                        if shifted=$(translate_shifted "$keyname"); then
                            keyname="$shifted"
                        fi
                        shift_active=0
                    fi
                    buffer+="$keyname"
                    ;;
            esac
        elif [[ "$line" == *"value 0"* ]]; then
            if [[ "$keyname" == "LSHIFT" || "$keyname" == "RSHIFT" ]]; then
                shift_active=0
            fi
        fi
    done
}

while true; do
    if [[ ! -e "$EVENT_DEV" ]]; then
        echo "Device $EVENT_DEV not found. Waiting..."
        sleep "$RETRY_DELAY"
        continue
    fi

    echo "Reading from $EVENT_DEV (Ctrl+C to stop)..."
    run_event_loop
    echo "Event loop exited unexpectedly. Restarting in ${RETRY_DELAY}s..."
    sleep "$RETRY_DELAY"
done
