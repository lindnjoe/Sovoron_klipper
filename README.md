# Klipper configuration

This repository captures the complete Klipper setup that drives a CoreXY toolchanger equipped with six toolheads, two OpenAMS units attached to individual toolheads, a four lane boxturtle used as a lane loader for four individual toolheads, and Armored Turtle's Automated Filament Control (AFC) stack. The tree bundles the live `printer_data` configuration, the Python extensions that power AFC/OpenAMS coordination, and upstream reference material so the machine definition can be rebuilt or adapted without hunting through multiple sources.

## Repository layout

- `printer_data/config/` – Active Klipper configuration for the printer, including the main `printer.cfg`, the StealthChanger style toolchanger definition, and per-tool includes for the six CAN-based toolheads. The directory also contains dedicated folders for AFC (`AFC/`), OpenAMS (`oamsc.cfg` and `oams_macros.cfg`), and supporting macros such as crash detection, homing, and user utilities.
- `AFC-Klipper-Add-On/extras/` – Armored Turtle's AFC Python modules that Klipper loads to orchestrate spool handling, Moonraker integration, lane state tracking, and LED feedback for the automation hardware.
- `klipper_openams/src/` – Custom OpenAMS manager that supervises the dual AMS units, including runout detection, stuck-spool recovery, clog monitoring, and encoder sanity checks tailored to this installation.
- `klipper/klippy/extras/` – Patched Klipper extras that bridge AFC lane objects with OpenAMS hubs so sensor state, LEDs, and tool synchronisation stay in lockstep.
- `klipper-toolchanger-easy/examples/` – Upstream examples kept as reference material for the toolchanger macros, probing strategies, and slicer snippets used while evolving this build.

## Getting started

1. Copy the desired Python extras into your Klipper environment. The files under `AFC-Klipper-Add-On/extras/`, `klipper/klippy/extras/`, and `klipper_openams/src/` replace or extend the stock modules, so they should live alongside your printer's `klippy/extras` tree before restarting Klipper.
2. Place the contents of `printer_data/config/` into the host's `~/printer_data/config/` directory (or symlink it) so Klipper loads the same configuration structure seen here. Update `canbus_uuid` values, pin mappings, and offsets to match your hardware before enabling the printer to prevent CAN bus or docking collisions.
3. Restart Klipper and Moonraker. The AFC extras register event handlers and webhook endpoints at startup, so a clean restart ensures Moonraker surfaces the AFC status pages and the OpenAMS manager initialises its monitors correctly.

## Key configuration areas

### Toolchanger definition
The `toolchanger.cfg` file contains the StealthChanger-inspired motion paths, pickup/drop-off sequences, and helper macros used during calibration. It defines safe Y clearances, verifies tool detection during pickup, and exposes macros such as `TOOL_ALIGN_START/TEST/DONE` and the overridden `RESUME` routine so the toolchanger is always initialised before printing resumes.

Each CAN toolhead inherits its parking coordinates, input shaper tuning, accelerometer wiring, and detection switch mapping from the corresponding `tool-EBBT*.cfg` include. Update the coordinates and offsets there whenever a dock is realigned.

### AFC configuration
The `AFC/` folder defines lane speeds, LED behaviour, macro sequencing (load, poop, wipe, etc.), and MCU wiring for the Box Turtle and Pico expansion boards. Defaults for material temperatures, density, assisted unload behaviour, and tip-form routines live in `AFC.cfg`, while `AFC_Macro_Vars.cfg` and the `macros/` directory hold the user-facing G-code blocks that wrap AFC operations. The Python module `AFC.py` ties those settings back into Klipper, registering Moonraker endpoints, state machines, and save/resume hooks so AFC can pause prints, light LEDs, and sync spool data automatically.

### OpenAMS management
`oamsc.cfg` declares both OpenAMS MCUs, the filament pressure sensors, and macro variables used to position the cutter, calibrate PTFE lengths, and map spool groups to tool numbers. It also enables optional extruder inlet sensors and sets PID targets for hub rewind behaviour. On the firmware side, `oams_manager.py` maintains the runout monitors, stuck-spool detection, clog tracking, and post-load pressure validation that protect prints and highlight the offending lane when something goes wrong.

## Updating for your machine

- Edit the CAN UUIDs for every MCU (`printer.cfg`, `oamsc.cfg`, and each `tool-EBBT*.cfg`) after flashing new hardware.
- Adjust the tool dock coordinates (`params_park_*`) and offsets in each tool file whenever a dock is moved or adjusted.
- Calibrate the OpenAMS `ptfe_length`, `hub_hes_on`, and FPS thresholds in `oamsc.cfg` after any tubing or hub maintenance so the manager loads and unloads reliably.
- Tune AFC speeds, LED colours, and macro toggles in `AFC.cfg` to match your filament handling preferences before running long jobs.

## Additional references

The `klipper-toolchanger-easy/examples/` folder retains the upstream documentation for common probing setups, dock mechanisms, and slicer macros that inspired the current toolchanger implementation. Consult it when experimenting with alternative docking paths or slicer start/end code.
