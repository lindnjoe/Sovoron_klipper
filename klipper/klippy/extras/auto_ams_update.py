import logging

SYNC_INTERVAL = 1.0

class AutoAMSUpdate:
    """Periodically update virtual pins from AMS lane status."""

    def __init__(self, config):
        self.printer = config.get_printer()
        self.reactor = self.printer.get_reactor()
        self.gcode = self.printer.lookup_object('gcode')
        self.interval = config.getfloat('interval', SYNC_INTERVAL, above=0.)
        self.oams1_name = config.get('oams1', 'oams1').strip()
        self.oams2_name = config.get('oams2', 'oams2').strip()
        self.pin_names = config.getlist(
            'pins',
            (
                'pin1','pin2','pin3','pin4','pin5','pin6','pin7','pin8',
                'pin9','pin10','pin11','pin12','pin13','pin14','pin15','pin16',
            ))
        if len(self.pin_names) != 16:
            raise config.error('pins option must contain sixteen pin names')
        self.timer = self.reactor.register_timer(self._sync_event)
        self.printer.register_event_handler('klippy:ready', self.handle_ready)

    def handle_ready(self, eventtime=None):
        self.reactor.update_timer(self.timer, self.reactor.NOW)

    def _sync_event(self, eventtime):
        try:
            oams1 = self.printer.lookup_object('oams ' + self.oams1_name, None)
            oams2 = self.printer.lookup_object('oams ' + self.oams2_name, None)
            vals1 = getattr(oams1, 'f1s_hes_value', [0,0,0,0]) if oams1 else [0,0,0,0]
            vals2 = getattr(oams2, 'f1s_hes_value', [0,0,0,0]) if oams2 else [0,0,0,0]
            hubs1 = getattr(oams1, 'hub_hes_value', [0,0,0,0]) if oams1 else [0,0,0,0]
            hubs2 = getattr(oams2, 'hub_hes_value', [0,0,0,0]) if oams2 else [0,0,0,0]
            def update_pin(name, value):
                cmdline = f"SET_VIRTUAL_PIN PIN={name} VALUE={int(value)}"
                self.gcode.run_script_from_command(cmdline)
            # f1s_hes values -> pins1..8
            for i in range(4):
                pin = self.pin_names[i]
                update_pin(pin, vals1[i])
            for i in range(4):
                pin = self.pin_names[i+4]
                update_pin(pin, vals2[i])
            # hub_hes values -> pins9..16
            for i in range(4):
                pin = self.pin_names[i+8]
                update_pin(pin, hubs1[i])
            for i in range(4):
                pin = self.pin_names[i+12]
                update_pin(pin, hubs2[i])
        except Exception:
            logging.exception('auto AMS update error')
        return eventtime + self.interval

def load_config(config):
    return AutoAMSUpdate(config)