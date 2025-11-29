# OpenAMS component responsibilities

This repository ships multiple OpenAMS-related modules that run side by side. They share concepts (lanes, sensors, spool metadata) but serve distinct runtimes, so both files remain necessary:

- **`klipper/klippy/extras/oams_manager.py`** – the Klipper module that owns runout/clog/stuck-spool monitoring, pause/resume behavior, and direct MCU coordination. It interacts with printers via the Klipper extension API and keeps timers/reactor tasks alive even when AFC is unavailable.
- **`AFC-Klipper-Add-On/extras/AFC_OpenAMS.py`** – the AFC integration layer. It adapts OpenAMS events into AFC’s lane registry, virtual sensors, spool metadata application (material/color/Spoolman IDs), and cross-extruder runout coordination. It depends on AFC’s helpers and is loaded with the AFC add-on rather than Klipper’s module loader.
- **`klipper_openams/src/oams_manager.py`** – a byte-for-byte backup of the primary manager. It stays in sync so legacy paths continue to work without diverging behavior or needing import shims.

All three entry points remain because some installations still reference the legacy `klipper_openams` path, but the backup copy keeps runtime behavior identical while avoiding drift. The Klipper runtime and AFC add-on still evolve independently while sharing utilities (for example, `openams_integration.py`), but there is now a single manager implementation that both paths share.
