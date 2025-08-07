"""Chip manager emulating MCU behavior for virtual pins."""

import logging
import pins


class VirtualPinChip:
    """Chip manager emulating MCU behavior for virtual pins."""

    def __init__(self, printer):
        self.printer = printer
        self.pins = {}
        # store (handler, [vpin objects]) for buttons_state callbacks
        self._response_handlers = []
        self._ack_count = 0
        self._config_callbacks = []
        self.printer.register_event_handler('klippy:ready',
                                            self._run_config_callbacks)

    def add_pin(self, vpin):
        if vpin.name in self.pins:
            raise pins.error(f"Duplicate virtual_pin {vpin.name}")
        index = len(self.pins)
        self.pins[vpin.name] = (vpin, index)
        if self._response_handlers:
            for handler, vpins in self._response_handlers:
                if vpin in vpins:
                    vpin.register_watcher(
                        lambda val, h=handler, p=vpins: self._pin_changed(h, p))
                    self._pin_changed(handler, vpins)

    def _pin_changed(self, handler, vpins):
        state = 0
        for i, vpin in enumerate(vpins):
            if vpin.state:
                state |= 1 << i
        params = {
            'ack_count': self._ack_count & 0xff,
            'state': bytes([state]),
            '#receive_time': self.printer.get_reactor().monotonic(),
        }
        self._ack_count += 1
        try:
            handler(params)
        except Exception:
            logging.exception('Virtual pin chip handler error')

    def setup_pin(self, pin_type, pin_params):
        pin_name = pin_params['pin']
        entry = self.pins.get(pin_name)
        if entry is None:
            raise pins.error('virtual_pin %s not configured' % (pin_name,))
        vpin, idx = entry
        return vpin._setup_pin(pin_type, pin_params)

    # --------------------------------------------------------------
    # Minimal MCU interface used by modules like buttons.py
    # --------------------------------------------------------------
    def register_config_callback(self, cb):
        self._config_callbacks.append(cb)

    def _run_config_callbacks(self, eventtime=None):
        for cb in self._config_callbacks:
            try:
                cb()
            except Exception:
                logging.exception('Virtual pin chip config callback error')
        self._config_callbacks = []

    def create_oid(self):
        return 0

    def add_config_cmd(self, cmd, is_init=False, on_restart=False):
        pass

    class _DummyCmd:
        def send(self, params):
            pass

    def alloc_command_queue(self):
        return None

    def lookup_command(self, template, cq=None):
        return self._DummyCmd()

    def get_query_slot(self, oid):
        return 0

    def seconds_to_clock(self, time):
        return 0

    def register_response(self, handler, resp_name=None, oid=None):
        if resp_name != 'buttons_state':
            return
        # obtain pin order from MCU_buttons object if available
        pin_list = getattr(getattr(handler, '__self__', None), 'pin_list', [])
        if not pin_list:
            pin_list = getattr(handler, 'pin_list', [])
        vpins = []
        for pin_name, _pullup in pin_list:
            entry = self.pins.get(pin_name)
            if entry is None:
                logging.error('virtual pin %s not configured', pin_name)
                continue
            vpins.append(entry[0])
        if not vpins:
            return
        self._response_handlers.append((handler, vpins))
        for vpin in vpins:
            vpin.register_watcher(
                lambda val, h=handler, p=vpins: self._pin_changed(h, p))
        self._pin_changed(handler, vpins)


def load_config(config):
    """Create and register a VirtualPinChip."""
    printer = config.get_printer()
    chip = VirtualPinChip(printer)
    add_obj = getattr(printer, 'add_object', None)
    if add_obj is not None:
        add_obj('virtual_pin_chip', chip)
    else:
        printer.objects['virtual_pin_chip'] = chip
    try:
        printer.lookup_object('pins').register_chip('virtual_pin', chip)
    except pins.error:
        pass
    return chip
