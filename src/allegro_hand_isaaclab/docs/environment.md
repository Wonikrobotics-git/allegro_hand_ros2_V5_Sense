# Environment Reference

**English** | [한국어](environment.kr.md) — [← README](../README.md)

## 1. The demo script, `run_sim.py`

The entry point that runs without installing anything. It bypasses Gym and `DirectRLEnv`, using
`SimulationContext` and `Articulation` directly to spawn the hands and loop a trajectory from `data/`.

```bash
python run_sim.py [--side left|right|both] [--traj wave|grasp] [--speed 1.0] [--max_steps N]
```

| Option | Default | Description |
|---|---|---|
| `--side` | `both` | Which hands to spawn. A single hand is centred on the origin |
| `--traj` | `wave` | Trajectory name under `data/`, or a path to a `.npy` file |
| `--speed` | `1.0` | Playback speed multiplier |
| `--max_steps` | `None` | Stop after this many steps. Without it, runs until the window closes |

Standard Isaac Lab options (`--device`, `--headless`, `--visualizer`, …) are accepted as well.
There is no stopping condition without a viewer, so pass `--max_steps` for headless runs.

```bash
python run_sim.py --visualizer none --max_steps 300
```

### Trajectory data format

Each `data/*.npy` is a `(T, 16)` float32 array holding **flexion in radians from the neutral pose**
(always ≥ 0). `run_sim.py` converts it to absolute targets per hand:

```python
target = clamp(neutral + direction * delta, lower, upper)
```

`direction` is the sign (±1) pointing from neutral towards whichever position limit lies further
away. Applying **only the direction** per hand — never the magnitude — is the important part. The
left and right assets do not share joint ranges (`joint_3_0` spans 1.86 rad on the right but
2.01 rad on the left; the left `joint_13_0` only 1.35 rad), so a normalised `[0, 1]` trajectory would
map to different angles on each hand and the motion would visibly diverge. This scheme also absorbs
the left thumb's sign flip and the ~π offset on `joint_13_0` (see [asset_notes.md](asset_notes.md)).

Regenerate the trajectories with:

```bash
python scripts/make_trajectories.py
```

Keep `MAX_FLEX` in `scripts/make_trajectories.py` (default 1.3 rad) below the tightest range the two
hands share — about 1.35 rad, set by the left `joint_13_0` — so playback never clamps and the hands
stay symmetric.

---

## 2. The training environment

| Item | Value |
|---|---|
| Task id | `Template-V5-Sense-Dual-Hand-Direct-v0` |
| Classes | `V5SenseDualHandEnv` / `V5SenseDualHandEnvCfg` (`DirectRLEnv`) |
| Robots | Right at `(0, -0.15, 0.2)`, left at `(0, +0.15, 0.2)` — both fixed-base, fingers up |
| Action space | **32** = `[right joint_0_0..joint_15_0, left joint_0_0..joint_15_0]` |
| Action mapping | `[-1, 1]` → each joint's position limits, EMA-smoothed → position target |
| Observation space | **64** = `[R pos(16), R vel(16)×0.2, L pos(16), L vel(16)×0.2]` |
| Reward | `0.0` (placeholder) |
| Termination | Time limit only (`episode_length_s = 10.0`) |
| Default env count | 64 (override with `--num_envs`) |
| decimation / dt | 4 / `1/120` s |

Running it:

```bash
# random actions (Kit viewer)
python scripts/random_agent.py --task=Template-V5-Sense-Dual-Hand-Direct-v0 --num_envs=4

# zero actions (check the neutral pose holds)
python scripts/zero_agent.py --task=Template-V5-Sense-Dual-Hand-Direct-v0 --num_envs=4

# headless smoke test
python scripts/random_agent.py --task=Template-V5-Sense-Dual-Hand-Direct-v0 \
    --num_envs=8 --headless --visualizer none --max_steps 200
```

### Tuning knobs

All in `tasks/direct/allegro_v5_sense/dual_hand_env_cfg.py`, and overridable from the CLI through
Hydra (`env.action_scale=0.5`).

| Field | Default | Description |
|---|---|---|
| `action_scale` | `1.0` | How much of each joint's range an action covers |
| `act_moving_average` | `0.3` | EMA coefficient on position targets. `1.0` disables smoothing |
| `vel_obs_scale` | `0.2` | Joint velocity scaling in the observation |
| `reset_joint_pos_noise` | `0.05` rad | Joint position noise applied on reset |
| `decimation` | `4` | Physics steps per policy step |
| `episode_length_s` | `10.0` | Episode length in seconds |

Actuator gains (`stiffness=3.0`, `damping=0.1`, `effort_limit_sim=0.5`) live in `_actuators()` in
`assets/allegro_hand_v5_sense.py`.

---

## 3. Joint names and ordering

Each hand has 16 revolute joints, `joint_0_0` through `joint_15_0`.

| Finger | Joint indices |
|---|---|
| Index | 0 – 3 |
| Middle | 4 – 7 |
| Ring | 8 – 11 |
| Thumb | 12 – 15 |

For the index, middle and ring fingers the first joint (0, 4, 8) is abduction/adduction and the
remaining three are flexion. On the thumb, `joint_12_0` is opposition rotation, so its axis and
limits differ from the other fingers.

Beyond those there are four fixed fingertip joints and a `global` fixed joint pinning `palm_link` to
the world — which is what makes the hand fixed-base. The `link_sensor_*` prims are tactile sensor
sites (`IsaacSiteAPI`), not rigid bodies, so they do not appear in `body_names`. Each hand has 21
rigid bodies.

> **Ordering guarantee**
> The order comes from resolving the `ALLEGRO_HAND_V5_SENSE_JOINT_NAMES` list through
> `find_joints(..., preserve_order=True)`, not from PhysX's internal joint ordering. The
> action/observation index layout therefore always matches the table above, whatever physics
> backend is in use. Downstream pipelines can rely on it.

---

## 4. Orientation and dimensions

The hands spawn **upright**: the four fingers point along `+Z` and the thumb along `-Y` on the
right hand / `+Y` on the left, so the two are exact mirrors of one another about the `XZ` plane.
Palms face `-X`.

The URDF-converted asset itself points the fingers along `-Z`. Both configs apply `UPRIGHT_ROT`
— a 180° rotation about `Y` — to stand them up. `Y` is used rather than `X` because it flips the
finger direction while leaving the thumb's `±Y` direction alone, which is what preserves the mirror
symmetry. `palm_link` sits at `PALM_HEIGHT` (0.2 m), putting the lowest point of the hand at about
0.12 m above the ground.

| Measurement | Value |
|---|---|
| Hand width (Y) | 0.204 m |
| Hand length (Z) | 0.222 m |
| Palm → fingertip | 0.140 m |
| Palm → thumb tip | 0.181 m |
| Vertical extent | z ∈ [0.117, 0.340] m |

For a hand mounted on the end of an arm you may want the original fingers-down pose instead — set
`ALLEGRO_HAND_V5_SENSE_*_CFG.init_state.rot` to the identity `(1, 0, 0, 0)`.

---

## 5. Asset paths

Asset paths resolve relative to the repository root. If you move the USD files elsewhere, point the
`V5_SENSE_ASSET_DIR` environment variable at the directory containing `v5_sense_left/` and
`v5_sense_right/`.

```bash
export V5_SENSE_ASSET_DIR=/shared/assets/v5_sense
```
