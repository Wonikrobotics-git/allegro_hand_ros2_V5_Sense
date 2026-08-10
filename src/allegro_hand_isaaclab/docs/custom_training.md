# Building a Task on Top

**English** | [한국어](custom_training.kr.md) — [← README](../README.md)

The base environment has a reward of `0`, so training it as-is learns nothing. That is deliberate:
the point is to hand you a scene where both hands are known to load and step correctly, and let you
put the task on top.

---

## Option A. Edit the environment in place

Fastest. Fill in three methods in `tasks/direct/allegro_v5_sense/dual_hand_env.py`.

```python
def _get_rewards(self) -> torch.Tensor:
    # returns shape (num_envs,)
    action_penalty = torch.sum(self.actions**2, dim=-1)
    return -self.cfg.action_penalty_scale * action_penalty

def _get_dones(self) -> tuple[torch.Tensor, torch.Tensor]:
    time_out = self.episode_length_buf >= self.max_episode_length - 1
    terminated = <your success/failure condition>      # shape (num_envs,), dtype=bool
    return terminated, time_out

def _get_observations(self) -> dict:
    # if you add object poses here, bump observation_space in dual_hand_env_cfg.py to match
    ...
```

Put scales and thresholds in `dual_hand_env_cfg.py` as fields rather than hard-coding them — that
makes them overridable from the CLI via Hydra (`env.action_penalty_scale=0.01`).

---

## Option B. Subclass (recommended)

Keeps this repository usable as an upstream: register a new task without touching the base scene.

```python
# tasks/direct/my_task/my_task_env.py
from v5_sense_isaaclab_pipeline.tasks.direct.allegro_v5_sense.dual_hand_env import V5SenseDualHandEnv

class MyTaskEnv(V5SenseDualHandEnv):
    def _get_rewards(self):
        ...
    def _get_dones(self):
        ...
```

```python
# tasks/direct/my_task/my_task_env_cfg.py
from isaaclab.utils.configclass import configclass
from v5_sense_isaaclab_pipeline.tasks.direct.allegro_v5_sense.dual_hand_env_cfg import V5SenseDualHandEnvCfg

@configclass
class MyTaskEnvCfg(V5SenseDualHandEnvCfg):
    observation_space = 64 + 7      # e.g. when adding an object pose
    my_reward_scale = 1.0
```

```python
# tasks/direct/my_task/__init__.py
import gymnasium as gym
from . import agents

gym.register(
    id="Template-V5-Sense-MyTask-Direct-v0",
    entry_point=f"{__name__}.my_task_env:MyTaskEnv",
    disable_env_checker=True,
    kwargs={
        "env_cfg_entry_point": f"{__name__}.my_task_env_cfg:MyTaskEnvCfg",
        "skrl_cfg_entry_point": f"{agents.__name__}:skrl_ppo_cfg.yaml",
    },
)
```

`tasks/__init__.py` imports subpackages automatically, so no extra wiring is needed — create the
directory with an `__init__.py` and `scripts/list_envs.py` will pick it up. Under `agents/`, put an
`__init__.py` and a skrl yaml (copy the existing task's).

> To drop the `Template-` prefix from task ids, also change the `"Template-"` search pattern in
> `scripts/list_envs.py` or they will stop showing up in the listing.

---

## Adding objects

Create the object in `_setup_scene()` **before** `clone_environments()`, and register it under
`self.scene.rigid_objects[...]`.

```python
def _setup_scene(self):
    self.right_hand = Articulation(self.cfg.right_hand_cfg)
    self.left_hand = Articulation(self.cfg.left_hand_cfg)
    self.object = RigidObject(self.cfg.object_cfg)      # <- added
    spawn_ground_plane(prim_path="/World/ground", cfg=GroundPlaneCfg())
    self.scene.clone_environments(copy_from_source=False)
    if self.device == "cpu":
        self.scene.filter_collisions(global_prim_paths=[])
    self.scene.articulations["right_hand"] = self.right_hand
    self.scene.articulations["left_hand"] = self.left_hand
    self.scene.rigid_objects["object"] = self.object    # <- added
    ...
```

`prim_path` must follow the `/World/envs/env_.*/<name>` form or the object will not be cloned per
environment. Reset the object pose in `_reset_idx()` too, adding `self.scene.env_origins[env_ids]`
to its position so each environment places it correctly.

The hands spawn fingers-up with their palms facing `-X`, so an object placed slightly towards `-X`
and around `z = 0.3` sits in front of the palms. Adjust `PALM_HEIGHT` and the per-hand `init_state`
in `assets/allegro_hand_v5_sense.py` to suit your task.

---

## Training

```bash
# train
python scripts/skrl/train.py --task=Template-V5-Sense-Dual-Hand-Direct-v0 \
    --num_envs=4096 --headless --max_iterations=1000

# replay a checkpoint
python scripts/skrl/play.py --task=Template-V5-Sense-Dual-Hand-Direct-v0 \
    --num_envs=16 --checkpoint=<path/to/checkpoint.pt>
```

Useful flags: `--seed`, `--video` (with `--video_length`, `--video_interval`), and `--checkpoint` to
resume.

Hyperparameters live in `tasks/direct/allegro_v5_sense/agents/skrl_ppo_cfg.yaml` (network
`[256, 128, 64]`). Logs are written under `logs/skrl/v5_sense_dual_hand_direct/`.

For another library (rsl_rl, rl_games, …), add the matching `*_cfg_entry_point` to the `gym.register`
kwargs, then copy Isaac Lab's training script for it into `scripts/` and add the single line
`import v5_sense_isaaclab_pipeline.tasks`.

---

## Adding demo trajectories

Separately from training, to play back a specific motion: add a generator function to
`scripts/make_trajectories.py` and register it in `main()`. Format and units are described in
[environment.md](environment.md#trajectory-data-format).

```bash
python scripts/make_trajectories.py
python run_sim.py --traj <name>
```

If you have joint trajectories recorded on real hardware, convert them to flexion-from-neutral in
radians and save as a `(T, 16)` array — they will play back as-is.
