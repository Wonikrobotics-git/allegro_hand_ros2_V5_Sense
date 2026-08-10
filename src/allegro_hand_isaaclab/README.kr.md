# Allegro Hand V5 Sense — Isaac Lab Pipeline

[English](README.md) | **한국어**

![Isaac Lab](https://img.shields.io/badge/Isaac%20Lab-3.0-76b900) ![Isaac Sim](https://img.shields.io/badge/Isaac%20Sim-6.0-76b900) ![Python](https://img.shields.io/badge/python-3.12-3776ab)

Wonik Robotics **Allegro Hand V5 "sense"(촉각) 좌·우 손**을 Isaac Lab에 스폰하고 학습까지 이어갈 수 있는 프로젝트입니다.
바로 실행 가능한 데모(`run_sim.py`)와, 그 위에 task를 얹을 수 있는 강화학습 환경을 함께 제공합니다.

<p align="center">
  <img src="docs/media/wave.gif" width="480" alt="양손 wave 궤적 재생">
</p>

<p align="center"><sub>양손이 순차적으로 손가락을 굽히는 <code>wave</code> 궤적 — 전체 영상은 <a href="docs/media/demo.mp4">docs/media/demo.mp4</a></sub></p>

---

## Quick Start

Isaac Lab 3.0이 설치된 python으로 실행합니다. **설치 과정 없이** 바로 손이 움직이는 걸 볼 수 있습니다.

```bash
python run_sim.py                 # 양손, wave 궤적
python run_sim.py --side right    # 오른손만
python run_sim.py --traj grasp    # 쥐었다 펴기
```

Isaac Lab 자체 설치는 [공식 가이드](https://isaac-sim.github.io/IsaacLab/main/source/setup/installation/index.html)를 따르세요.
이 레포는 `IsaacLab` 디렉터리 **바깥에** 두면 됩니다.

학습 환경까지 쓰려면 확장 패키지를 설치합니다.

```bash
python -m pip install -e source/v5_sense_isaaclab_pipeline

python scripts/list_envs.py                                                    # 등록 확인
python scripts/random_agent.py --task=Template-V5-Sense-Dual-Hand-Direct-v0 --num_envs=4
python scripts/skrl/train.py   --task=Template-V5-Sense-Dual-Hand-Direct-v0 --num_envs=4096 --headless
```

---

## Repository Structure

```text
├── run_sim.py                     // 데모 진입점 (설치 불필요, gym 미사용)
├── data/                          // 재생용 궤적 (wave.npy, grasp.npy)
├── asset/
│   ├── v5_sense_left/             // 왼손 USD (URDF 변환)
│   └── v5_sense_right/            // 오른손 USD
├── scripts/
│   ├── random_agent.py            // 랜덤 액션
│   ├── zero_agent.py              // 0 액션
│   ├── list_envs.py               // 등록된 task 목록
│   ├── make_trajectories.py       // data/*.npy 생성
│   └── skrl/{train,play}.py       // skrl PPO 학습 / 재생
├── source/v5_sense_isaaclab_pipeline/
│   └── v5_sense_isaaclab_pipeline/
│       ├── assets/                // ArticulationCfg (좌/우 손)
│       └── tasks/direct/allegro_v5_sense/   // DirectRLEnv 환경
└── docs/                          // 상세 문서
```

---

## 두 개의 진입점

| | `run_sim.py` | `Template-V5-Sense-Dual-Hand-Direct-v0` |
|---|---|---|
| 용도 | 손이 제대로 로드·동작하는지 확인 | 강화학습 |
| 설치 | 불필요 | `pip install -e` 필요 |
| 구성 | `SimulationContext` + `Articulation` 직접 사용 | Gym 등록된 `DirectRLEnv` |
| 동작 | `data/*.npy` 궤적 재생 | 정책이 관절 목표를 출력 |
| 손 | `--side left\|right\|both` | 양손 고정 |

학습 환경 요약 — action 32(양손 16관절씩), observation 64(관절 위치·속도), **reward는 0인 placeholder**입니다.
task를 얹는 방법은 [docs/custom_training.kr.md](docs/custom_training.kr.md)를 보세요.

---

## 문서

| 문서 | 내용 |
|---|---|
| [docs/environment.kr.md](docs/environment.kr.md) | 환경 사양, 관절 매핑, 자세·크기, 실행 옵션 |
| [docs/custom_training.kr.md](docs/custom_training.kr.md) | reward·종료조건 작성, 물체 추가, 새 task 등록, 학습 실행 |
| [docs/asset_notes.kr.md](docs/asset_notes.kr.md) | URDF→USD 변환 특성과 우회 처리 (**왼손 엄지 미러링 문제 포함**) |
| [docs/troubleshooting.kr.md](docs/troubleshooting.kr.md) | 자주 나는 오류, IDE·Omniverse 확장 설정 |

에셋을 다시 export하거나 손 설정을 바꿀 계획이라면 [docs/asset_notes.kr.md](docs/asset_notes.kr.md)를 먼저 읽어보시길 권합니다.
