"""
OpenAMS Spoolman LED Sync Module

Optional module that sets the ACTIVE TOOL LED to match Spoolman filament color
instead of the default blue color. Non-active lanes use their normal AFC colors.

Requires AFC and Spoolman integration to be active.

Configuration:
[spoolman_led_sync]
enable: True  # Set to False to disable
default_color: 0000FF  # Hex color for lanes without Spoolman data (default: blue)
"""

import logging

class SpoolmanLEDSync:
    def __init__(self, printer, config):
        self.printer = printer
        self.name = config.get_name()
        self.logger = logging.getLogger(self.name)

        # Configuration
        self.enabled = config.getboolean('enable', False)
        self.default_color = config.get('default_color', '0000FF')

        if not self.enabled:
            self.logger.info("Spoolman LED sync disabled")
            return

        self.logger.info("Spoolman LED sync enabled")

        # Defer AFC lookup until ready
        self.afc = None

        try:
            # Register for multiple events to catch whenever AFC becomes available
            self.logger.info("Registering event handlers...")
            self.printer.register_event_handler("klippy:ready", self._handle_ready)
            self.logger.info("Registered klippy:ready")
            self.printer.register_event_handler("klippy:connect", self._handle_ready)
            self.logger.info("Registered klippy:connect")

            # Also schedule delayed init in case AFC loads after this module
            self.reactor = printer.get_reactor()
            self.logger.info("Got reactor, scheduling callback...")
            self.reactor.register_callback(self._delayed_init)
            self.logger.info("Registered for ready/connect events and scheduled delayed init")
        except Exception as e:
            self.logger.exception("Failed to register event handlers")

    def _delayed_init(self, eventtime):
        """Try to connect after a short delay, in case AFC loads after us"""
        self.logger.info("Delayed init callback triggered")
        self._handle_ready()

    def _handle_ready(self):
        """Initialize AFC connection after Klipper is ready"""
        # Skip if already connected
        if self.afc is not None:
            self.logger.debug("Already connected to AFC, skipping re-init")
            return

        self.logger.info("Attempting to connect to AFC...")
        try:
            self.afc = self.printer.lookup_object('AFC')
            self.logger.info("Successfully looked up AFC object")

            if not hasattr(self.afc, 'function'):
                self.logger.error("AFC object has no 'function' attribute")
                self.enabled = False
                return

            self.logger.info("AFC has function attribute, attempting hook...")

            # Register for toolhead activation events
            # AFC calls handle_activate_extruder when lanes change
            self._hook_into_afc()

            self.logger.info("Hook complete, module fully initialized")

        except Exception as e:
            self.logger.exception("Failed to connect to AFC")
            self.enabled = False

    def _hook_into_afc(self):
        """
        Hook into AFC's lane activation system without modifying AFC code.
        We wrap the existing handle_activate_extruder function.
        """
        try:
            afc_function = self.afc.function
            original_activate = afc_function.handle_activate_extruder

            def wrapped_activate_extruder():
                """Call original function, then update LEDs with Spoolman colors"""
                # Let AFC do its normal activation
                original_activate()

                # Now override LED colors for loaded lanes if we have Spoolman data
                self._update_lane_leds()

            # Replace the function with our wrapper
            afc_function.handle_activate_extruder = wrapped_activate_extruder
            self.logger.info("Successfully hooked into AFC lane activation")

        except Exception as e:
            self.logger.exception("Failed to hook into AFC activation: %s", e)

    def _update_lane_leds(self):
        """Update LEDs for all lanes based on Spoolman colors"""
        self.logger.info("_update_lane_leds called (enabled=%s, afc=%s)", self.enabled, self.afc is not None)

        if not self.enabled or not self.afc:
            self.logger.warning("Skipping LED update - not enabled or no AFC")
            return

        try:
            lane_count = len(self.afc.lanes) if hasattr(self.afc, 'lanes') else 0
            self.logger.info("Updating LEDs for %d lanes", lane_count)

            for lane_name, lane in self.afc.lanes.items():
                self._set_lane_led_color(lane)
        except Exception as e:
            self.logger.exception("Error updating lane LEDs")

    def _set_lane_led_color(self, lane):
        """
        Set the active tool's LED to its Spoolman color instead of default blue.
        Only updates the lane that is currently loaded in the toolhead.
        """
        lane_name = getattr(lane, 'name', 'unknown')

        # Skip if lane has no LED
        if not hasattr(lane, 'led_index') or lane.led_index is None:
            self.logger.debug("Lane %s: No LED index, skipping", lane_name)
            return

        # Only set color for loaded lanes
        prep = hasattr(lane, 'prep_state') and lane.prep_state
        load = hasattr(lane, 'load_state') and lane.load_state

        if not (prep and load):
            self.logger.debug("Lane %s: Not loaded (prep=%s, load=%s), skipping", lane_name, prep, load)
            return

        # Check if this is the active toolhead lane
        is_active = (hasattr(lane, 'extruder_obj') and lane.extruder_obj is not None and
                     hasattr(lane.extruder_obj, 'lane_loaded') and
                     lane.extruder_obj.lane_loaded == lane.name)

        self.logger.info("Lane %s: loaded, is_active=%s", lane_name, is_active)

        if is_active:
            # For the active tool, override blue with Spoolman color
            hex_color = self._get_lane_color(lane)
            led_color_str = self._hex_to_led_string(hex_color)

            self.logger.info("Setting active tool %s LED to color %s (string: %s)",
                           lane_name, hex_color, led_color_str)

            try:
                self.afc.function.afc_led(led_color_str, lane.led_index)
                self.logger.info("Successfully set active tool LED for %s to color %s", lane_name, hex_color)
            except Exception as e:
                self.logger.exception("Failed to set LED for %s", lane_name)

    def _get_lane_color(self, lane):
        """
        Get the hex color for a lane from Spoolman data.
        Returns default color if Spoolman data not available.
        """
        # Try to get color from lane (set by Spoolman)
        if hasattr(lane, 'color') and lane.color:
            hex_color = lane.color
            # Remove # prefix if present
            if hex_color.startswith('#'):
                hex_color = hex_color[1:]
            return hex_color

        # Fallback to default blue
        return self.default_color

    def _hex_to_led_string(self, hex_color):
        """
        Convert hex color to AFC LED string format (R,G,B,W as floats 0-1).
        Uses AFC's existing HexToLedString if available, otherwise implements it.
        """
        try:
            # Try to use AFC's built-in converter
            if hasattr(self.afc.function, 'HexToLedString'):
                led_values = self.afc.function.HexToLedString(hex_color)
                # Convert list to comma-separated string
                return ','.join(str(v) for v in led_values)
        except Exception as e:
            self.logger.debug("Couldn't use AFC's HexToLedString, using fallback: %s", e)

        # Fallback implementation
        try:
            # Parse hex: RRGGBB
            if len(hex_color) >= 6:
                r = int(hex_color[0:2], 16) / 255.0
                g = int(hex_color[2:4], 16) / 255.0
                b = int(hex_color[4:6], 16) / 255.0
            else:
                # Invalid hex, use default
                r, g, b = 0.0, 0.0, 1.0  # Blue

            # Add white channel (0 unless pure white)
            w = 1.0 if hex_color.upper() == "FFFFFF" else 0.0

            return f"{r},{g},{b},{w}"
        except Exception as e:
            self.logger.error("Failed to convert hex %s: %s", hex_color, e)
            return "0,0,1,0"  # Blue fallback

def load_config(config):
    return SpoolmanLEDSync(config.get_printer(), config)