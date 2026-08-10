# 트러블슈팅 & 개발 환경 설정

[English](troubleshooting.md) | **한국어** — [← README](../README.kr.md)

## 자주 나는 오류

### `KeyError: 'EXP_PATH'` 또는 `NameError: name 'isaacsim' is not defined`

Isaac Sim 환경변수가 셸에 없는 상태입니다. Isaac Lab 런처를 쓰거나 환경을 먼저 source하세요.

```bash
# 방법 1 — Isaac Lab 런처
<ISAACLAB_PATH>/isaaclab.sh -p run_sim.py

# 방법 2 — 환경을 셸에 source
source <ISAACSIM_PATH>/setup_conda_env.sh
python run_sim.py
```

### `Could not find the Allegro Hand V5 sense (...) USD file at: ...`

에셋 경로 해석에 실패한 경우입니다. `V5_SENSE_ASSET_DIR`를 `v5_sense_left/`와 `v5_sense_right/`가
들어 있는 디렉터리로 지정하세요.

```bash
cd <ros2_workspace>/src/allegro_hand_isaaclab
export V5_SENSE_ASSET_DIR=$PWD/asset
```

**이 저장소에서는 반드시 지정해야 합니다.** `assets/__init__.py`가 자기 파일 기준으로 네 단계 위를
에셋 경로로 잡는데, 이는 원본 standalone 레이아웃
(`source/v5_sense_isaaclab_pipeline/v5_sense_isaaclab_pipeline/assets/`)을 전제한 계산입니다.
ROS 2 워크스페이스의 `src/allegro_hand_isaaclab/` 아래에 들어온 지금 구조에서는 기본값이
존재하지 않는 `<ros2_workspace>/asset`을 가리킵니다. 매번 치기 번거로우면 셸 프로필에 넣어두세요.

패키지를 editable(`pip install -e`)이 아닌 방식으로 설치한 경우에도 발생합니다. 에셋이 설치된
패키지 바깥에 남기 때문입니다.

### headless 실행이 끝나지 않음

`--max_steps N`을 주세요. 뷰어가 없으면 종료 조건이 없습니다.
`run_sim.py`, `random_agent.py`, `zero_agent.py` 모두 이 옵션을 지원하며,
없이 실행하면 무한 루프 대신 에러로 막습니다.

### `RuntimeError: Explicitly requested visualizer(s) ['kit'] could not be configured`

`--headless`와 Kit 뷰어를 동시에 요구해서 나는 에러입니다. `--visualizer none`을 함께 주세요.

```bash
python run_sim.py --headless --visualizer none --max_steps 300
```

### `ValueError: The following joints have default positions out of the limits: 'joint_13_0'`

왼손 엄지의 중립값이 잘못 설정된 경우입니다. `LEFT_THUMB_ROT_JOINT_NEUTRAL`이
`[1.780, 3.140]` 범위 안에 있어야 합니다. 자세한 배경은
[asset_notes.md](asset_notes.kr.md#1-왼손-엄지-joint_13_0이-미러링되어-있지-않음)를 보세요.

### `RuntimeError: Expected exactly one ArticulationRootAPI prim ... found 2`

`ArticulationRootPropertiesCfg`에 `fix_root_link=True`를 준 경우입니다. 이 에셋에는 이미
`global` fixed joint가 있어서 설정하면 안 됩니다.
[asset_notes.md](asset_notes.kr.md#2-fix_root_linktrue를-쓰면-안-됨) 참고.

---

## 개발 환경 설정

### VSCode

`Ctrl+Shift+P` → `Tasks: Run Task` → `setup_python_env` 실행. Isaac Sim 설치 경로를 입력하면
`.vscode/.python.env`가 생성되어 Omniverse 모듈 자동완성이 동작합니다.

### 코드 포맷팅

```bash
pip install pre-commit
pre-commit run --all-files
```

ruff(lint + format)가 설정되어 있습니다.

### Omniverse 익스텐션으로 로드

1. `Window` → `Extensions` → 햄버거 아이콘 → `Settings`
2. `Extension Search Paths`에 이 프로젝트를 **담고 있는** 디렉터리의 절대경로를 추가.
   `src/allegro_hand_isaaclab`이 아니라 `<ros2_workspace>/src`입니다. Kit은 한 단계 아래에서
   `config/extension.toml`을 가진 폴더를 찾는데, 이 파일이
   `src/allegro_hand_isaaclab/config/extension.toml`에 있기 때문입니다.
   (없다면 `IsaacLab/source`도 함께 추가)
3. 햄버거 아이콘 → `Refresh`
4. `Third Party` 카테고리에서 익스텐션을 찾아 활성화

UI 익스텐션 예제는 `v5_sense_isaaclab_pipeline/ui_extension_example.py`에 있습니다.

### Pylance 인덱싱 누락

`.vscode/settings.json`의 `"python.analysis.extraPaths"`에 경로를 추가하세요.

```json
{
    "python.analysis.extraPaths": [
        "<ros2_workspace>/src/allegro_hand_isaaclab"
    ]
}
```

### Pylance 크래시

인덱싱 대상이 너무 많아 메모리가 부족한 경우입니다. `"python.analysis.extraPaths"`에서
사용하지 않는 omniverse 패키지를 주석 처리하세요.

```json
"<path-to-isaac-sim>/extscache/omni.anim.*"         // 애니메이션
"<path-to-isaac-sim>/extscache/omni.kit.*"          // Kit UI
"<path-to-isaac-sim>/extscache/omni.graph.*"        // Graph UI
"<path-to-isaac-sim>/extscache/omni.services.*"     // 서비스
```
