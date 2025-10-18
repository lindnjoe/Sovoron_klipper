# Copilot Instructions for Sovoron Klipper

## Repository Overview

This repository contains the complete Klipper configuration for a CoreXY toolchanger printer with six toolheads, two OpenAMS units, and Armored Turtle's Automated Filament Control (AFC) stack. The repository combines live printer configuration, Python extensions, and reference materials.

## Code Organization

### Directory Structure

- **`printer_data/config/`**: Active Klipper configuration files
  - `printer.cfg` - Main printer configuration
  - `toolchanger.cfg` - StealthChanger toolchanger definition
  - `tool-EBBT*.cfg` - Per-toolhead configuration (6 CAN-based toolheads)
  - `AFC/` - AFC configuration and macros
  - `oamsc.cfg`, `oams_macros.cfg` - OpenAMS configuration
  - `homing.cfg`, `speed.cfg`, `debug.cfg` - Supporting configurations

- **`AFC-Klipper-Add-On/extras/`**: Python modules for AFC functionality
  - `AFC.py` - Main AFC orchestration module
  - `AFC_*.py` - Supporting modules for stepper control, spool handling, LED feedback, etc.

- **`klipper_openams/src/`**: Custom OpenAMS manager Python modules
  - `oams_manager.py` - Main manager for dual AMS units
  - `oams.py` - OpenAMS control logic
  - `fps.py` - Filament pressure sensor handling
  - `filament_group.py` - Spool group management

- **`klipper/klippy/extras/`**: Patched Klipper extras for AFC-OpenAMS integration

- **`klipper-toolchanger-easy/examples/`**: Upstream reference material

## Development Guidelines

### Configuration Files (`.cfg`)

- Klipper configuration files use INI-style syntax with `[section_name]` blocks
- Configuration changes should preserve existing coordinate systems and hardware mappings
- CAN bus UUIDs (`canbus_uuid`) are hardware-specific and should not be changed without verification
- Tool dock coordinates (`params_park_*`) and offsets are calibrated values - handle with care
- Always maintain comments that document hardware-specific calibrations

### Python Modules

- Follow Klipper's plugin architecture conventions
- Python modules in `extras/` directories are loaded by Klipper at startup
- Modules register with Klipper's event system and expose G-code commands
- Moonraker integration is done through webhook endpoints
- Use Klipper's logging system (`self.gcode.respond_info()`) for user-visible messages
- State management should be thread-safe for Klipper's async event model

### Key Concepts

**Toolchanger Operations:**
- Tool pickup/drop-off sequences must maintain safe Y clearances
- Tool detection switches verify successful docking
- Each tool has independent input shaper tuning and accelerometer configuration

**AFC (Automated Filament Control):**
- Lane-based spool management system
- Integrates with Moonraker for status reporting
- LED feedback for visual lane status
- Coordinates with OpenAMS for filament loading

**OpenAMS:**
- Manages dual AMS units with runout detection
- Stuck-spool recovery and clog monitoring
- Pressure sensor validation after loading
- Per-hub rewind behavior with PID control

## Code Style

### Configuration Files
- Use lowercase for section names: `[stepper_x]`, not `[Stepper_X]`
- Indent parameter continuation lines consistently
- Group related settings together with blank lines between groups
- Document non-obvious values with inline comments

### Python Code
- Follow PEP 8 style guidelines
- Use descriptive variable names that reflect Klipper terminology
- Maintain existing error handling patterns
- Log important state changes for debugging

## Safety Considerations

- **CAN Bus Configuration**: Incorrect UUIDs can cause hardware conflicts
- **Motion Coordinates**: Wrong dock positions can cause crashes
- **Sensor Thresholds**: Incorrect values can lead to failed prints or hardware damage
- **Temperature Settings**: Verify PID values before committing changes

## Testing Approach

This repository does not have automated tests. Changes should be validated by:
1. Verifying Klipper syntax with configuration file checks
2. Testing Python modules by restarting Klipper and checking logs
3. Manual verification of toolchanger operations
4. Monitoring AFC and OpenAMS behavior during test prints

## Common Tasks

### Adding a New Configuration Parameter
1. Add to appropriate `.cfg` file in `printer_data/config/`
2. Update related macro files if needed
3. Document hardware-specific values

### Modifying Python Modules
1. Update module in appropriate `extras/` directory
2. Restart Klipper to load changes
3. Check Klipper logs for errors
4. Test G-code commands through console

### Updating Tool Configuration
1. Modify the specific `tool-EBBT*.cfg` file
2. Update dock coordinates if repositioned
3. Verify CAN bus UUID matches hardware
4. Test tool pickup/drop-off sequences

## Important Notes

- This is a configuration repository, not application source code
- Most "code" is declarative Klipper configuration
- Python modules extend Klipper's functionality
- Hardware-specific values are calibrated for this specific printer
- Changes should be minimal and focused on the requested modification
- Always preserve working configurations unless explicitly changing them
