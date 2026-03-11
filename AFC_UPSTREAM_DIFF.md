# AFC upstream comparison (OpenAMS-focused, after reset-command generalization)

Compared local tree `AFC-Klipper-Add-On/extras` against upstream branch `multi_extruder` from:

- Repo: `https://github.com/lindnjoe/AFC-Klipper-Add-On`
- Branch: `multi_extruder`
- Local clone used for comparison: `/workspace/AFC-upstream-multi`

## What changed in this update

- `extras/AFC_functions.py` reset handling was generalized from OpenAMS-specific checks to unit-level custom reset commands.
- Any unit implementing `get_lane_reset_command(lane, dis)` now participates in reset prompt and lane-reset flow without hardcoding unit type checks.

## High-level diff summary (`extras/` only)

- 10 files differ.
- `+62 / -326` lines total.

## Exact file-level differences vs upstream

### Modified files

1. `extras/AFC_functions.py` (`+29/-27`)
2. `extras/AFC_stepper.py` (`+14/-3`)
3. `extras/AFC_buffer.py` (`+13/-2`)
4. `extras/AFC_OpenAMS.py` (`+1/-1`)
5. `extras/AFC_Toolchanger.py` (`+1/-1`)
6. `extras/AFC_error.py` (`+1/-1`)
7. `extras/AFC_hub.py` (`+1/-1`)
8. `extras/AFC_lane.py` (`+1/-1`)
9. `extras/openams_integration.py` (`+1/-1`)

### Present upstream but missing locally

- `extras/AFC_vivid.py` (`-288` lines relative to upstream)

## Commands used

```bash
git clone --depth 1 --branch multi_extruder https://github.com/lindnjoe/AFC-Klipper-Add-On /workspace/AFC-upstream-multi
git diff --no-index --name-status /workspace/AFC-upstream-multi/extras /workspace/Sovoron_klipper/AFC-Klipper-Add-On/extras
git diff --no-index --stat /workspace/AFC-upstream-multi/extras /workspace/Sovoron_klipper/AFC-Klipper-Add-On/extras
git diff --no-index --numstat /workspace/AFC-upstream-multi/extras /workspace/Sovoron_klipper/AFC-Klipper-Add-On/extras | sort -nr
```
