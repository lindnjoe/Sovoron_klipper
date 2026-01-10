# Klipper configuration snapshot

This repository tracks the live Klipper and Moonraker setup that powers a CoreXY StealthChanger-style toolchanger with six CAN toolheads, dual OpenAMS units, and Armored Turtle's Automated Filament Control (AFC) stack. It combines the configuration under `printer_data/`, the custom Python extras that coordinate AFC/OpenAMS behaviour, companion services such as the QR-based spool scanner, and upstream references used while developing the machine. Cloning the repo provides everything required to recreate or audit the current machine definition without needing to chase multiple sources.

## Repository layout

- `printer_data/config/` – Authoritative Klipper configuration. Besides the main `printer.cfg`, the directory contains:
  - `toolchanger/` and `toolchanger-*.cfg` files for StealthChanger motion, pickup/drop-off sequences, LED choreography, and user macros.
  - `tool-EBBT*.cfg`, `tool-AVR*.cfg`, and matching `*-leds.cfg` includes that define each toolhead's CAN UUIDs, parking coordinates, probe offsets, heater tuning, and RGB feedback.
  - `AFC/` with macros, variables, and speed/temperature presets that govern the AFC lane loader, poop/wipe sequences, and assisted unload routines.
  - Service configuration like `moonraker.conf`, `octoeverywhere*.cfg`, `crowsnest.conf`, `KNOMI.cfg`, and `shell_command.cfg` so the host-side integrations mirror the printer.
  - Sensor and motion tuning files (`homing.cfg`, `smart_filament_sensor.cfg`, `speed.cfg`, `ShakeTune_results/`, etc.) that keep input shaping, runout detection, and crash detection aligned with the hardware.
- `AFC-Klipper-Add-On/extras/` – AFC Python modules dropped into Klipper's `klippy/extras/`. They implement lane state machines, Moonraker endpoints, LED updates, and save/resume hooks that tie the macros in `printer_data/config/AFC/` to the firmware runtime.
- `klipper_openams/src/` – Custom OpenAMS manager responsible for dual-hub monitoring, pressure validation, stuck-spool recovery, and encoder sanity checks tailored to this build.
- `klipper/klippy/extras/` – Patched Klipper extras that expose AFC lane objects to the OpenAMS manager and keep tool state, LEDs, and filament sensors in sync.
- `afc-spool-scan/` – Systemd service (`usb-qr-scanner.service`) and helper script (`usb-qr-scanner-read.sh`) that watch a USB QR scanner and push spool metadata into Moonraker/AFC when a tag is scanned.
- `klipper-toolchanger-easy/examples/` – Upstream reference macros and documentation for toolchanger probing, dock mechanisms, and slicer snippets used while iterating on this machine.

## Deploying the configuration

1. **Copy the configuration tree** – Mirror `printer_data/config/` to the host running Klipper (usually `~/printer_data/config/`). Update CAN UUIDs, heater/thermistor types, dock offsets, accelerometer IDs, and probe locations in the per-tool files before enabling steppers.
2. **Install the Python extras** – Drop the contents of `AFC-Klipper-Add-On/extras/`, `klipper/klippy/extras/`, and `klipper_openams/src/` into the corresponding directories of your Klipper checkout (`klippy/extras/` and `klippy/extras/openams/` or similar). Restart Klipper so the custom modules register their event handlers and REST endpoints.
3. **Configure companion services** –
   - Enable the QR spool scanner by copying `afc-spool-scan/usb-qr-scanner-read.sh` and `afc-spool-scan/usb-qr-scanner.service` to the host, adjust the script's serial device path, then run `systemctl enable --now usb-qr-scanner`.
   - Review `moonraker.conf`, `octoeverywhere.conf`, `crowsnest.conf`, and other service files to align API keys, hostnames, and camera devices with your environment.
4. **Restart Klipper and Moonraker** – A clean restart ensures AFC/OpenAMS modules initialise correctly, registers Moonraker UI panels, and activates the filament automation workflows.

## Key configuration highlights

### Toolchanger and motion control
- `toolchanger.cfg` and the `toolchanger-*.cfg` files define the StealthChanger motion envelopes, safe clearances, dock detection, and helper macros (`TOOL_ALIGN_*`, `DOCK_*`, `RESUME`, etc.) used during calibration and recovery.
- Per-tool includes inherit shared motion parameters but override parking coordinates, nozzle offsets, accelerometer buses, and LED behaviours. Update these whenever docks are realigned or new tools are added.

### Automated Filament Control (AFC)
- `AFC.cfg`, `AFC_Macro_Vars.cfg`, and the `macros/` directory coordinate lane speeds, unload/load routines, LED colours, purge lengths, and save/resume hooks.
- `AFC.py` (in `AFC-Klipper-Add-On/extras/`) keeps Moonraker status pages, QR spool metadata, and AFC lane state machines in sync so the printer can pause, prompt, or resume automatically during filament changes.

### OpenAMS integration
- `oamsc.cfg` describes both AMS MCUs, pressure sensors, cutter macros, PID targets, optional inlet sensors, and spool group assignments for the dual-hub setup.
- `oams_manager.py` (under `klipper_openams/src/`) monitors runout, stuck spools, clog detection, and post-load pressure validation, surfacing alerts to Moonraker and coordinating with AFC macros.
- Load/unload retries use a fixed delay per attempt (configured in `printer_data/config/AFC/AFC_oams.cfg` via `retry_backoff_base`, capped by `retry_backoff_max`) rather than an exponential backoff.

## Adapting to other machines

- Replace CAN UUIDs, heater IDs, tool parking coordinates, and probe offsets across `printer.cfg`, each `tool-*.cfg`, and `oamsc.cfg` to match your hardware before attempting tool changes.
- Tune AFC lane speeds, temperature presets, tip-form routines, and LED colour choices in `printer_data/config/AFC/` to match your filament handling preferences.
- Re-measure PTFE lengths, hub PID settings, and filament sensor thresholds in `oamsc.cfg` after any maintenance to ensure the OpenAMS manager continues to load reliably.
- Align Moonraker- and OctoEverywhere-specific secrets with your host machine; the placeholders in the repo are tailored to the original installation.

## Additional references

The `klipper-toolchanger-easy/examples/` directory keeps the upstream documentation that informed this build. Consult it when experimenting with alternative probing strategies, docking sequences, or slicer start/end code.
