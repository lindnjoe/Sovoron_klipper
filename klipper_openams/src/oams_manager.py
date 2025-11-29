# OpenAMS Manager Shim
#
# This wrapper reuses the primary OpenAMS manager implementation that lives
# under `klipper/klippy/extras/oams_manager.py`. OpenAMS and AFC are always
# deployed together in this environment, so keeping a single implementation
# avoids divergence while preserving the legacy module entry point.

from __future__ import annotations

import importlib
import os
import sys
from types import ModuleType


def _load_primary_manager() -> ModuleType:
    """Import the main OpenAMS manager module and return it.

    The shim adjusts `sys.path` so installs that keep the AFC add-on and
    Klipper tree side by side can locate the shared implementation without
    bundling a second copy.
    """

    repo_root = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", ".."))
    extras_path = os.path.join(repo_root, "klipper", "klippy", "extras")

    if extras_path not in sys.path:
        sys.path.insert(0, extras_path)

    try:
        return importlib.import_module("oams_manager")
    except Exception as exc:  # pragma: no cover - defensive import wrapper
        raise ImportError(
            f"Unable to import primary OpenAMS manager from '{extras_path}'"
        ) from exc


# Load the shared implementation and replace this module with it so existing
# imports continue to work without carrying duplicate logic.
_PRIMARY = _load_primary_manager()
sys.modules[__name__] = _PRIMARY

