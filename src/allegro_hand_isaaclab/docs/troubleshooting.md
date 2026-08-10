# Troubleshooting & Development Setup

**English** | [한국어](troubleshooting.kr.md) — [← README](../README.md)

## Common errors

### `KeyError: 'EXP_PATH'` or `NameError: name 'isaacsim' is not defined`

The Isaac Sim environment variables are not set in your shell. Use the Isaac Lab launcher, or source
the environment first.

```bash
# option 1 — Isaac Lab launcher
<ISAACLAB_PATH>/isaaclab.sh -p run_sim.py

# option 2 — source the environment into your shell
source <ISAACSIM_PATH>/setup_conda_env.sh
python run_sim.py
```

### `Could not find the Allegro Hand V5 sense (...) USD file at: ...`

Asset path resolution failed. Point `V5_SENSE_ASSET_DIR` at the directory holding `v5_sense_left/`
and `v5_sense_right/`.

```bash
cd <ros2_workspace>/src/allegro_hand_isaaclab
export V5_SENSE_ASSET_DIR=$PWD/asset
```

**In this repository you always have to set it.** `assets/__init__.py` resolves the asset folder four
levels up from its own file, which assumes the upstream standalone layout
(`source/v5_sense_isaaclab_pipeline/v5_sense_isaaclab_pipeline/assets/`). Vendored into a ROS 2
workspace under `src/allegro_hand_isaaclab/`, that default lands on `<ros2_workspace>/asset`, which
does not exist. Add the `export` to your shell profile to make it permanent.

It also happens whenever the package is installed non-editable (without `pip install -e`), since the
assets then live outside the installed package.

### A headless run never finishes

Pass `--max_steps N`. Without a viewer there is no stopping condition. `run_sim.py`,
`random_agent.py` and `zero_agent.py` all support the flag, and raise an error rather than looping
forever if you omit it.

### `RuntimeError: Explicitly requested visualizer(s) ['kit'] could not be configured`

Caused by asking for `--headless` and the Kit viewer at the same time. Add `--visualizer none`.

```bash
python run_sim.py --headless --visualizer none --max_steps 300
```

### `ValueError: The following joints have default positions out of the limits: 'joint_13_0'`

The left thumb's neutral value is wrong. `LEFT_THUMB_ROT_JOINT_NEUTRAL` has to fall inside
`[1.780, 3.140]`. Background in
[asset_notes.md](asset_notes.md#1-the-left-thumbs-joint_13_0-is-not-mirrored).

### `RuntimeError: Expected exactly one ArticulationRootAPI prim ... found 2`

You passed `fix_root_link=True` in `ArticulationRootPropertiesCfg`. This asset already has a `global`
fixed joint, so the flag must stay unset. See
[asset_notes.md](asset_notes.md#2-do-not-set-fix_root_linktrue).

---

## Development setup

### VSCode

`Ctrl+Shift+P` → `Tasks: Run Task` → `setup_python_env`. Enter your Isaac Sim install path and it
generates `.vscode/.python.env`, which enables autocomplete for the Omniverse modules.

### Code formatting

```bash
pip install pre-commit
pre-commit run --all-files
```

ruff (lint + format) is configured.

### Loading as an Omniverse extension

1. `Window` → `Extensions` → hamburger icon → `Settings`
2. Add the absolute path of the directory **containing** this project to `Extension Search Paths` —
   i.e. `<ros2_workspace>/src`, not `src/allegro_hand_isaaclab` itself. Kit scans one level down for
   folders holding a `config/extension.toml`, and here that file is at
   `src/allegro_hand_isaaclab/config/extension.toml`. (Add `IsaacLab/source` too if it is not already
   there.)
3. Hamburger icon → `Refresh`
4. Find the extension under `Third Party` and enable it

The UI extension example is at `v5_sense_isaaclab_pipeline/ui_extension_example.py`.

### Pylance missing indexing

Add the path to `"python.analysis.extraPaths"` in `.vscode/settings.json`.

```json
{
    "python.analysis.extraPaths": [
        "<ros2_workspace>/src/allegro_hand_isaaclab"
    ]
}
```

### Pylance crash

Usually too many indexed files exhausting memory. Comment out unused omniverse packages under
`"python.analysis.extraPaths"`.

```json
"<path-to-isaac-sim>/extscache/omni.anim.*"         // animation
"<path-to-isaac-sim>/extscache/omni.kit.*"          // Kit UI
"<path-to-isaac-sim>/extscache/omni.graph.*"        // graph UI
"<path-to-isaac-sim>/extscache/omni.services.*"     // services
```
