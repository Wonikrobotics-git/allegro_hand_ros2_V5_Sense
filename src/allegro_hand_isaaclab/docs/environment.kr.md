# 환경 사양

[English](environment.md) | **한국어** — [← README](../README.kr.md)

## 1. 데모 스크립트 `run_sim.py`

설치 없이 동작하는 진입점입니다. Gym이나 `DirectRLEnv`를 거치지 않고 `SimulationContext`와
`Articulation`을 직접 써서, 손을 스폰하고 `data/`의 궤적을 반복 재생합니다.

```bash
python run_sim.py [--side left|right|both] [--traj wave|grasp] [--speed 1.0] [--max_steps N]
```

| 옵션 | 기본값 | 설명 |
|---|---|---|
| `--side` | `both` | 스폰할 손. 한 손만 고르면 원점에 가운데 정렬됩니다 |
| `--traj` | `wave` | `data/` 안의 궤적 이름, 또는 `.npy` 파일 경로 |
| `--speed` | `1.0` | 재생 속도 배수 |
| `--max_steps` | `None` | 지정한 스텝 후 종료. 없으면 창을 닫을 때까지 실행 |

Isaac Lab 공통 옵션(`--device`, `--headless`, `--visualizer` 등)도 그대로 받습니다.
화면 없이 돌릴 때는 종료 조건이 없으므로 `--max_steps`를 함께 주세요.

```bash
python run_sim.py --visualizer none --max_steps 300
```

### 궤적 데이터 형식

`data/*.npy`는 `(T, 16)` float32 배열이고, 값은 **중립 자세로부터의 굽힘량(radian, 항상 ≥ 0)**입니다.
`run_sim.py`가 손별로 다음과 같이 실제 목표 각도로 바꿉니다.

```python
target = clamp(neutral + direction * delta, lower, upper)
```

`direction`은 중립에서 더 먼 쪽 limit을 향하는 부호(±1)입니다. **크기가 아니라 방향만** 손마다
다르게 적용하는 게 핵심입니다. 좌우 에셋의 관절 가동범위가 서로 다르기 때문에
(예: `joint_3_0`이 오른손 1.86 rad / 왼손 2.01 rad, 왼손 `joint_13_0`은 1.35 rad),
정규화된 `[0,1]` 궤적을 쓰면 같은 값이 좌우에서 다른 각도로 매핑되어 동작이 어긋납니다.
이 방식은 왼손 엄지의 부호 반전과 `joint_13_0`의 ~π 오프셋도 함께 흡수합니다
([asset_notes.md](asset_notes.kr.md) 참고).

궤적 재생성:

```bash
python scripts/make_trajectories.py
```

`scripts/make_trajectories.py`의 `MAX_FLEX`(기본 1.3 rad)는 좌우가 공유하는 가장 좁은 가동범위
(약 1.35 rad, 왼손 `joint_13_0`이 결정)보다 작게 유지해야 클램핑 없이 좌우가 대칭으로 움직입니다.

---

## 2. 학습 환경

| 항목 | 값 |
|---|---|
| Task id | `Template-V5-Sense-Dual-Hand-Direct-v0` |
| 클래스 | `V5SenseDualHandEnv` / `V5SenseDualHandEnvCfg` (`DirectRLEnv`) |
| 로봇 | 오른손 `(0, -0.15, 0.2)`, 왼손 `(0, +0.15, 0.2)` — 둘 다 fixed base, 손가락 위 방향 |
| Action space | **32** = `[오른손 joint_0_0..joint_15_0, 왼손 joint_0_0..joint_15_0]` |
| Action 해석 | `[-1, 1]` → 각 관절의 position limit으로 매핑 후 EMA 스무딩 → position target |
| Observation space | **64** = `[R pos(16), R vel(16)×0.2, L pos(16), L vel(16)×0.2]` |
| Reward | `0.0` (placeholder) |
| 종료 | time limit만 (`episode_length_s = 10.0`) |
| 기본 env 수 | 64 (`--num_envs`로 override) |
| decimation / dt | 4 / `1/120` s |

실행:

```bash
# 랜덤 액션 (Kit 뷰어)
python scripts/random_agent.py --task=Template-V5-Sense-Dual-Hand-Direct-v0 --num_envs=4

# 0 액션 (기본 자세 유지 확인)
python scripts/zero_agent.py --task=Template-V5-Sense-Dual-Hand-Direct-v0 --num_envs=4

# headless 스모크 테스트
python scripts/random_agent.py --task=Template-V5-Sense-Dual-Hand-Direct-v0 \
    --num_envs=8 --headless --visualizer none --max_steps 200
```

### 튜닝 포인트

`tasks/direct/allegro_v5_sense/dual_hand_env_cfg.py`에서 바로 조정할 수 있습니다.
Hydra로 CLI override도 가능합니다 (`env.action_scale=0.5`).

| 필드 | 기본값 | 설명 |
|---|---|---|
| `action_scale` | `1.0` | 액션이 관절 range의 몇 배를 커버할지 |
| `act_moving_average` | `0.3` | position target EMA 계수. `1.0`이면 스무딩 없음 |
| `vel_obs_scale` | `0.2` | 관측의 관절 속도 스케일 |
| `reset_joint_pos_noise` | `0.05` rad | 리셋 시 관절 위치 노이즈 |
| `decimation` | `4` | 물리 스텝 / 정책 스텝 비율 |
| `episode_length_s` | `10.0` | 에피소드 길이 (초) |

액추에이터 게인(`stiffness=3.0`, `damping=0.1`, `effort_limit_sim=0.5`)은
`assets/allegro_hand_v5_sense.py`의 `_actuators()`에 있습니다.

---

## 3. 관절 이름과 순서

각 손은 revolute 관절 16개(`joint_0_0` ~ `joint_15_0`)를 가집니다.

| 손가락 | 관절 인덱스 |
|---|---|
| 검지 (index) | 0 – 3 |
| 중지 (middle) | 4 – 7 |
| 약지 (ring) | 8 – 11 |
| 엄지 (thumb) | 12 – 15 |

검지·중지·약지는 첫 관절(0, 4, 8)이 벌림·모음이고 나머지 셋이 굽힘입니다.
엄지는 `joint_12_0`이 대립(opposition) 회전이라 축과 limit이 다른 손가락과 다릅니다.

이 외에 fingertip fixed joint 4개와, `palm_link`를 월드에 고정하는 `global` fixed joint가 있습니다
(그래서 fixed base입니다). `link_sensor_*` prim은 촉각 센서 site(`IsaacSiteAPI`)이고 rigid body가
아니라서 `body_names`에는 나오지 않습니다. 강체는 손당 21개입니다.

> **관절 순서 보장**
> 순서는 PhysX 내부 정렬이 아니라 `ALLEGRO_HAND_V5_SENSE_JOINT_NAMES` 리스트를
> `find_joints(..., preserve_order=True)`로 resolve해서 결정됩니다. 즉 action/observation의
> 인덱스 레이아웃은 물리 백엔드가 바뀌어도 항상 위 표 그대로입니다. 외부 파이프라인에서
> 이 순서에 의존해도 안전합니다.

---

## 4. 방향과 크기

손은 **똑바로 선 자세**로 스폰됩니다. 네 손가락이 `+Z`(위)를 향하고 엄지는 오른손이 `-Y` /
왼손이 `+Y`라서, 두 손이 `XZ` 평면 기준으로 정확히 미러링됩니다. 손바닥은 `-X`를 향합니다.

URDF 변환 에셋 자체는 손가락이 `-Z`(아래)를 향합니다. 두 config 모두 `UPRIGHT_ROT`
(`Y`축 180° 회전)를 적용해 세웁니다. `X`축이 아니라 `Y`축인 이유는, 손가락 방향만 뒤집고
엄지의 `±Y` 방향은 그대로 두어야 좌우 미러 대칭이 유지되기 때문입니다. `palm_link`는
`PALM_HEIGHT`(0.2 m)에 놓이고, 손의 최하단은 지면에서 약 0.12 m입니다.

| 측정값 | 값 |
|---|---|
| 손 폭 (Y) | 0.204 m |
| 손 길이 (Z) | 0.222 m |
| 손바닥→손가락 끝 | 0.140 m |
| 손바닥→엄지 끝 | 0.181 m |
| 수직 범위 | z ∈ [0.117, 0.340] m |

로봇 팔 끝에 장착하는 용도라면 원래의 손가락 아래 자세가 나을 수 있습니다.
`ALLEGRO_HAND_V5_SENSE_*_CFG.init_state.rot`을 항등 `(1, 0, 0, 0)`으로 두세요.

---

## 5. 에셋 경로

에셋 경로는 레포 루트 기준으로 자동 해석됩니다. USD 파일을 다른 곳으로 옮겼다면
`V5_SENSE_ASSET_DIR` 환경변수를 `v5_sense_left/`와 `v5_sense_right/`가 들어 있는 디렉터리로
지정하세요.

```bash
export V5_SENSE_ASSET_DIR=/shared/assets/v5_sense
```
