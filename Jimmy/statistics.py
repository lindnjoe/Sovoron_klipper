# Support for logging periodic statistics
#
# Copyright (C) 2018-2021  Kevin O'Connor <kevin@koconnor.net>
#
# This file may be distributed under the terms of the GNU GPLv3 license.
import os, time, logging

class PrinterSysStats:
    def __init__(self, config):
        printer = config.get_printer()
        self.last_process_time = self.total_process_time = 0.
        self.last_load_avg = 0.
        self.last_mem_avail = 0
        self.mem_file = None
        try:
            self.mem_file = open("/proc/meminfo", "r")
        except:
            pass
        printer.register_event_handler("klippy:disconnect", self._disconnect)
    def _disconnect(self):
        if self.mem_file is not None:
            self.mem_file.close()
            self.mem_file = None
    def stats(self, eventtime):
        # Get core usage stats
        ptime = time.process_time()
        pdiff = ptime - self.last_process_time
        self.last_process_time = ptime
        if pdiff > 0.:
            self.total_process_time += pdiff
        self.last_load_avg = os.getloadavg()[0]
        msg = "sysload=%.2f cputime=%.3f" % (self.last_load_avg,
                                             self.total_process_time)
        # Get available system memory
        if self.mem_file is not None:
            try:
                self.mem_file.seek(0)
                data = self.mem_file.read()
                for line in data.split('\n'):
                    if line.startswith("MemAvailable:"):
                        self.last_mem_avail = int(line.split()[1])
                        msg = "%s memavail=%d" % (msg, self.last_mem_avail)
                        break
            except:
                pass
        return (False, msg)
    def get_status(self, eventtime):
        return {'sysload': self.last_load_avg,
                'cputime': self.total_process_time,
                'memavail': self.last_mem_avail}

class PrinterStats:
    def __init__(self, config):
        self.printer = config.get_printer()
        reactor = self.printer.get_reactor()
        self.stats_timer = reactor.register_timer(self.generate_stats)
        self.stats_cb = []
        self.printer.register_event_handler("klippy:ready", self.handle_ready)
        self.printer.register_event_handler("klippy:shutdown", self.handle_shutdown)
        self.shutdown_log_cnt = 50
        self.printer_is_ready = False
        self.stats_record_counter = 0
        self.stats_record_threshold = 2
        self.stats_record_threshold_in_paused = 30
        self.print_stats = None
        if config.has_section('printer'):
            printer_config = config.getsection('printer')
            self.stats_record_threshold = printer_config.getint(
                'stats_record_threshold', self.stats_record_threshold, minval=0)
            self.stats_record_threshold_in_paused = printer_config.getint(
                'stats_record_threshold_in_paused', self.stats_record_threshold_in_paused, minval=0)
    def handle_ready(self):
        self.shutdown_log_cnt = 50
        self.printer_is_ready = True
        self.stats_cb = [o.stats for n, o in self.printer.lookup_objects()
                         if hasattr(o, 'stats')]
        self.print_stats = self.printer.lookup_object('print_stats', None)
        if self.printer.get_start_args().get('debugoutput') is None:
            reactor = self.printer.get_reactor()
            reactor.update_timer(self.stats_timer, reactor.NOW)
    def handle_shutdown(self):
        self.printer_is_ready = False
    def generate_stats(self, eventtime):
        if not self.printer_is_ready:
            if self.shutdown_log_cnt > 0:
                logging.info("Printer is shutdown, final stats")
                self.shutdown_log_cnt -= 1
            else:
                return eventtime + 1.
        stats = [cb(eventtime) for cb in self.stats_cb]

        stats_record_threshold = self.stats_record_threshold
        if self.printer_is_ready and self.print_stats is not None and self.print_stats.state == 'paused':
            stats_record_threshold = self.stats_record_threshold_in_paused
        if max([s[0] for s in stats]):
            if stats_record_threshold > 0:
                self.stats_record_counter += 1
                if self.stats_record_counter >= stats_record_threshold:
                    logging.info("Stats %.1f: %s", eventtime,
                                ' '.join([s[1] for s in stats]))
                    self.stats_record_counter = 0
        return eventtime + 1.

def load_config(config):
    config.get_printer().add_object('system_stats', PrinterSysStats(config))
    return PrinterStats(config)
