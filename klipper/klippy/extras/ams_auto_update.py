import logging

SYNC_INTERVAL = 2.0

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
            class GC:
                def __init__(self, val):
                    self.val = val
                def get_int(self, name, default=None):
                    return self.val if name == 'VALUE' else default
            # f1s_hes values -> pins1..8
            for i in range(4):
                pin = self.pin_names[i]
                val = int(vals1[i])
                cmd = self.gcode.commands.get(('SET_VIRTUAL_PIN', pin))
                if cmd is not None:
                    cmd(GC(val))
            for i in range(4):
                pin = self.pin_names[i+4]
                val = int(vals2[i])
                cmd = self.gcode.commands.get(('SET_VIRTUAL_PIN', pin))
                if cmd is not None:
                    cmd(GC(val))
            # hub_hes values -> pins9..16
            for i in range(4):
                pin = self.pin_names[i+8]
                val = int(hubs1[i])
                cmd = self.gcode.commands.get(('SET_VIRTUAL_PIN', pin))
                if cmd is not None:
                    cmd(GC(val))
            for i in range(4):
                pin = self.pin_names[i+12]
                val = int(hubs2[i])
                cmd = self.gcode.commands.get(('SET_VIRTUAL_PIN', pin))
                if cmd is not None:
                    cmd(GC(val))
        except Exception:
            logging.exception('auto AMS update error')
        return eventtime + self.interval

def load_config(config):
    return AutoAMSUpdate(config)