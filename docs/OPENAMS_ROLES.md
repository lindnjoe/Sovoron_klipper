# OpenAMS component responsibilities

This repository ships multiple OpenAMS-related modules that work together to integrate OpenAMS with AFC while keeping legacy paths usable:

- **`klipper/klippy/extras/oams_manager.py`** – the primary Klipper module that owns runout/clog/stuck-spool monitoring, pause/resume behavior, and direct MCU coordination. This is the source of truth for OpenAMS control logic.
- **`klipper_openams/src/oams_manager.py`** – a byte-for-byte backup of the primary manager. It exists only so legacy configurations that import from `klipper_openams` keep running. It must stay identical to the main copy and should not diverge.
- **`AFC-Klipper-Add-On/extras/AFC_OpenAMS.py`** – the AFC integration layer. It forwards OpenAMS events into AFC’s lane registry, applies lane metadata (material/color/Spoolman IDs), and handles cross-extruder runout coordination using AFC helpers. This layer remains separate because it runs inside the AFC add-on rather than Klipper’s module loader.

The goal is to minimize duplication: the two manager files are mirrors, and AFC-specific behavior lives in the AFC add-on hook. All OpenAMS deployments are expected to run alongside AFC, but the legacy path remains for users that still reference the backup module.
