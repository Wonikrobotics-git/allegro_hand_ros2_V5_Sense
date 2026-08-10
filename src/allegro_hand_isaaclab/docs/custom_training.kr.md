# 커스텀 학습 붙이기

[English](custom_training.md) | **한국어** — [← README](../README.kr.md)

기본 환경은 reward가 0이라 그대로 학습해도 아무것도 배우지 않습니다.
"양손이 제대로 로드되고 스텝이 돈다"가 검증된 씬을 제공하는 것이 목적이고, task는 그 위에 얹습니다.

---

## 방법 A. 기존 환경을 직접 수정

가장 빠릅니다. `tasks/direct/allegro_v5_sense/dual_hand_env.py`의 세 메서드만 채우면 됩니다.

```python
def _get_rewards(self) -> torch.Tensor:
    # 반환 shape: (num_envs,)
    action_penalty = torch.sum(self.actions**2, dim=-1)
    return -self.cfg.action_penalty_scale * action_penalty

def _get_dones(self) -> tuple[torch.Tensor, torch.Tensor]:
    time_out = self.episode_length_buf >= self.max_episode_length - 1
    terminated = <실패/성공 조건>            # shape: (num_envs,), dtype=bool
    return terminated, time_out

def _get_observations(self) -> dict:
    # 물체 pose 등을 추가했다면 여기서 concat하고,
    # dual_hand_env_cfg.py의 observation_space 값도 같이 늘려야 합니다
    ...
```

스케일 값이나 임계값 같은 상수는 하드코딩하지 말고 `dual_hand_env_cfg.py`에 필드로 추가하세요.
Hydra로 CLI에서 override할 수 있게 됩니다 (`env.action_penalty_scale=0.01`).

---

## 방법 B. 서브클래싱 (권장)

이 레포를 업스트림으로 유지하면서 task만 새로 등록하는 방식입니다. 기본 씬을 건드리지 않습니다.

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
    observation_space = 64 + 7      # 물체 pose 추가 시
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

`tasks/__init__.py`가 하위 패키지를 자동으로 import하므로 별도 등록 코드는 필요 없습니다.
디렉터리를 만들고 `__init__.py`만 두면 `scripts/list_envs.py`에 바로 잡힙니다.
`agents/` 하위에는 `__init__.py`와 skrl yaml을 두면 됩니다 (기존 task의 것을 복사).

> task id에서 `Template-` 접두어를 빼려면 `scripts/list_envs.py`의 검색 패턴 `"Template-"`도
> 같이 바꿔야 목록에 나옵니다.

---

## 물체 추가하기

조작 대상을 넣으려면 `_setup_scene()`에서 `clone_environments()` **이전에** 생성하고,
`self.scene.rigid_objects[...]`에 등록하세요.

```python
def _setup_scene(self):
    self.right_hand = Articulation(self.cfg.right_hand_cfg)
    self.left_hand = Articulation(self.cfg.left_hand_cfg)
    self.object = RigidObject(self.cfg.object_cfg)      # <- 추가
    spawn_ground_plane(prim_path="/World/ground", cfg=GroundPlaneCfg())
    self.scene.clone_environments(copy_from_source=False)
    if self.device == "cpu":
        self.scene.filter_collisions(global_prim_paths=[])
    self.scene.articulations["right_hand"] = self.right_hand
    self.scene.articulations["left_hand"] = self.left_hand
    self.scene.rigid_objects["object"] = self.object    # <- 추가
    ...
```

`prim_path`는 반드시 `/World/envs/env_.*/<이름>` 형태여야 env마다 복제됩니다.
`_reset_idx()`에서 물체 pose도 함께 리셋해야 하며, 위치에 `self.scene.env_origins[env_ids]`를
더해야 env별로 올바른 자리에 놓입니다.

손은 손가락이 위, 손바닥이 `-X`를 향한 자세로 스폰됩니다. 물체를 `-X` 쪽으로 조금 옮기고
`z = 0.3` 근처에 두면 손바닥 앞에 놓입니다. `assets/allegro_hand_v5_sense.py`의 `PALM_HEIGHT`와
손별 `init_state`로 task에 맞게 조정하세요.

---

## 학습 실행

```bash
# 학습
python scripts/skrl/train.py --task=Template-V5-Sense-Dual-Hand-Direct-v0 \
    --num_envs=4096 --headless --max_iterations=1000

# 재생
python scripts/skrl/play.py --task=Template-V5-Sense-Dual-Hand-Direct-v0 \
    --num_envs=16 --checkpoint=<path/to/checkpoint.pt>
```

주요 옵션: `--seed`, `--video` (+ `--video_length`, `--video_interval`), `--checkpoint`(이어서 학습).

하이퍼파라미터는 `tasks/direct/allegro_v5_sense/agents/skrl_ppo_cfg.yaml`에서 수정합니다
(네트워크 `[256, 128, 64]`). 로그는 `logs/skrl/v5_sense_dual_hand_direct/` 아래에 쌓입니다.

skrl 외의 라이브러리(rsl_rl, rl_games 등)를 쓰려면 `gym.register`의 `kwargs`에 해당
`*_cfg_entry_point`를 추가하고, Isaac Lab의 해당 학습 스크립트를 `scripts/` 아래로 복사해
`import v5_sense_isaaclab_pipeline.tasks` 한 줄만 추가하면 됩니다.

---

## 데모 궤적 추가하기

학습과 별개로, 특정 동작을 재생해 보고 싶다면 `scripts/make_trajectories.py`에 생성 함수를
추가하고 `main()`에 등록하세요. 형식과 단위는 [environment.md](environment.kr.md#궤적-데이터-형식)를
참고하세요.

```bash
python scripts/make_trajectories.py
python run_sim.py --traj <이름>
```

실제 하드웨어에서 기록한 관절 궤적이 있다면, 중립 대비 굽힘량(rad)으로 변환해
`(T, 16)` 배열로 저장하면 그대로 재생됩니다.
