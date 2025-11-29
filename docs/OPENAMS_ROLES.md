# OpenAMS component responsibilities

This repository ships multiple OpenAMS-related modules that run side by side. They share concepts (lanes, sensors, spool metadata) but serve distinct runtimes, so both files remain necessary:

- **`klipper/klippy/extras/oams_manager.py`** – the Klipper module that owns runout/clog/stuck-spool monitoring, pause/resume behavior, and direct MCU coordination. It interacts with printers via the Klipper extension API and keeps timers/reactor tasks alive even when AFC is unavailable.
- **`AFC-Klipper-Add-On/extras/AFC_OpenAMS.py`** – the AFC integration layer. It adapts OpenAMS events into AFC’s lane registry, virtual sensors, spool metadata application (material/color/Spoolman IDs), and cross-extruder runout coordination. It depends on AFC’s helpers and is loaded with the AFC add-on rather than Klipper’s module loader.

Keeping both modules allows the Klipper runtime and AFC add-on to evolve independently while sharing utilities (for example, `openams_integration.py`). Consolidating them would remove the ability to run the Klipper monitoring logic without AFC present, and vice versa, so they intentionally stay as separate entry points even though they collaborate.
