#!/usr/bin/env python3

import configparser
import logging
import logging.handlers
import os
import runpy
import sys


LOG_FILE = "/oem/printer_data/logs/openrfid.log"
MAX_LOG_BYTES = 10 * 1024 * 1024  # 10 MB
MAX_BACKUPS = 1  # keep 2 total: current + 1 backup


def main():
    if len(sys.argv) < 2:
        print(f"Usage: {sys.argv[0]} <target.cfg> [source.cfg ...]")
        sys.exit(1)

    os.makedirs(os.path.dirname(LOG_FILE), exist_ok=True)

    syslogHandler = logging.handlers.SysLogHandler(address="/dev/log")
    stderrHandler = logging.StreamHandler(sys.stderr)
    fileHandler = logging.handlers.RotatingFileHandler(
        LOG_FILE,
        maxBytes=MAX_LOG_BYTES,
        backupCount=MAX_BACKUPS,
    )

    formatter = logging.Formatter(
        fmt="%(asctime)s %(levelname)s %(name)s: %(message)s",
        datefmt="%Y-%m-%d %H:%M:%S",
    )
    for handler in (syslogHandler, stderrHandler, fileHandler):
        handler.setFormatter(formatter)

    logging.root.handlers = [syslogHandler, stderrHandler, fileHandler]
    logging.root.setLevel(logging.INFO)

    # Suppress noisy loggers that spam on every RFID retry cycle
    logging.getLogger("urllib3").setLevel(logging.WARNING)
    logging.getLogger("urllib3.connectionpool").setLevel(logging.WARNING)
    # wakeup err / Scan error are expected when no tag is present
    logging.getLogger("rfid_reader").setLevel(logging.CRITICAL)

    target = sys.argv[1]
    sources = sys.argv[2:]

    config = configparser.RawConfigParser()
    config.optionxform = str
    config.read(sources)

    with open(target, "w") as f:
        config.write(f)

    sys.argv = [sys.argv[0], target]
    sys.path.insert(0, "/usr/local/share/openrfid")
    os.chdir("/usr/local/share/openrfid")
    runpy.run_path("main.py", run_name="__main__")


if __name__ == "__main__":
    main()
