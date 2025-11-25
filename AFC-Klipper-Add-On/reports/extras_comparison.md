# `extras/` comparison vs upstream `multi_extruder`

This report compares the current working tree's `AFC-Klipper-Add-On/extras` directory against `upstream/multi_extruder:extras` from https://github.com/lindnjoe/AFC-Klipper-Add-On/tree/multi_extruder.

## Summary
- Directories compared: `AFC-Klipper-Add-On/extras` (local) vs `extras` at `upstream/multi_extruder`.
- Result: all files have identical content **except** two modules that only differ by a missing trailing newline in the local copies.

## Detailed differences
| File | Status | Difference description |
| --- | --- | --- |
| `AFC_OpenAMS.py` | Differs | Content matches upstream aside from the local file missing the final newline character. |
| `openams_integration.py` | Differs | Content matches upstream aside from the local file missing the final newline character. |

All other files in `extras/` are byte-for-byte identical to the upstream `multi_extruder` branch.
