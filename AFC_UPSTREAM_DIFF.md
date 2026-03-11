# AFC upstream comparison (OpenAMS-focused, after merging e769f8f)

This report is the **post-merge** comparison after integrating upstream commit:

- `e769f8f019713ec34eab9cf0ff233c205658dd8d`
- "feat: add per-extruder variable overrides for AFC macros (#665)"

Compared local tree `AFC-Klipper-Add-On/extras` against upstream branch `multi_extruder` from:

- Repo: `https://github.com/lindnjoe/AFC-Klipper-Add-On`
- Branch: `multi_extruder`
- Local clone used for comparison: `/workspace/AFC-upstream-multi-extruder`

## What was merged from e769f8f into our tree

Given this repo snapshot only contains `AFC-Klipper-Add-On/extras`, the applicable change from `e769f8f` was in:

- `extras/AFC.py`

Specifically, tool-load/tool-unload macro invocations now pass per-extruder context with `EXTRUDER=<name>` for poop/brush/kick/cut/park macro calls.

## High-level diff summary (`extras/` only)

- 11 files differ.
- `+166 / -321` lines total.
- Relative to the previous comparison (`+173 / -328`), this merge reduced the delta by **7 insertions and 7 deletions**.

## Exact file-level differences after merge

### Modified files

1. `extras/AFC_functions.py` (`+50/-10`)
2. `extras/AFC.py` (`+29/-4`)
3. `extras/AFC_hub.py` (`+28/-6`)
4. `extras/AFC_lane.py` (`+28/-4`)
5. `extras/AFC_stepper.py` (`+14/-3`)
6. `extras/AFC_buffer.py` (`+13/-2`)
7. `extras/AFC_OpenAMS.py` (`+1/-1`)
8. `extras/AFC_Toolchanger.py` (`+1/-1`)
9. `extras/AFC_error.py` (`+1/-1`)
10. `extras/openams_integration.py` (`+1/-1`)

### Present upstream but missing locally

- `extras/AFC_vivid.py` (`-288` lines relative to upstream)

## Commands used

```bash
git clone --depth 1 --branch multi_extruder https://github.com/lindnjoe/AFC-Klipper-Add-On /workspace/AFC-upstream-multi-extruder
git fetch --deepen 200 origin multi_extruder
git show --name-status --oneline e769f8f019713ec34eab9cf0ff233c205658dd8d
git diff --no-index --name-status /workspace/AFC-upstream-multi-extruder/extras /workspace/Sovoron_klipper/AFC-Klipper-Add-On/extras
git diff --no-index --stat /workspace/AFC-upstream-multi-extruder/extras /workspace/Sovoron_klipper/AFC-Klipper-Add-On/extras
git diff --no-index --numstat /workspace/AFC-upstream-multi-extruder/extras /workspace/Sovoron_klipper/AFC-Klipper-Add-On/extras | sort -nr
```
