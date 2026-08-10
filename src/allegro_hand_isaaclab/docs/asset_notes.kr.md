# 에셋 관련 주의사항

[English](asset_notes.md) | **한국어** — [← README](../README.kr.md)

URDF → USD 변환 과정에서 생긴 특성들입니다. 코드에 우회 처리가 들어가 있으니
에셋을 다시 export할 때 참고하세요.

---

## 1. 왼손 엄지 `joint_13_0`이 미러링되어 있지 않음

왼손의 나머지 관절은 오른손과 대응되는데, **엄지 회전 관절 `joint_13_0`만** limit이
`[1.780, 3.140] rad` (`[102°, 180°]`)로 authoring되어 있습니다. 미러 값이라면
`[-1.780, 0.260]`이어야 합니다. `link_12_0`의 조인트 프레임이 미러 기준으로 약 π 회전되어 있기
때문입니다 (`localRot0`이 오른손과 약 180° 차이).

그 결과 이 관절의 중립값은 `0.0`이 아니라 **약 π**입니다. `0.0`으로 두면 Isaac Lab이
`"default positions out of the limits"` 에러로 articulation 자체를 거부합니다.

**우회 처리** — `assets/allegro_hand_v5_sense.py`:

```python
LEFT_THUMB_ROT_JOINT_NEUTRAL = 3.13
...
joint_pos={"^(?!joint_13_0$).*": 0.0, "joint_13_0": LEFT_THUMB_ROT_JOINT_NEUTRAL},
```

왼손 URDF를 제대로 미러링해서 다시 export하면 이 값을 `0.0`으로 되돌리면 됩니다.

### 부수 효과: 좌우 가동범위 불일치

이 문제 때문에 왼손 `joint_13_0`의 가동범위가 1.35 rad로, 오른손(1.78 rad)보다 좁습니다.
다른 관절에도 미세한 차이가 있습니다.

| 관절 | 오른손 range | 왼손 range |
|---|---|---|
| `joint_0_0` | `[-0.700, +0.262]` | `[-0.300, +0.300]` |
| `joint_3_0` / `7_0` / `11_0` | `[-0.100, +1.860]` | `[-0.100, +2.010]` |
| `joint_13_0` | `[-0.260, +1.780]` | `[+1.780, +3.140]` |
| `joint_14_0` | `[-0.050, +1.850]` | `[-1.780, +0.260]` |

그래서 데모 궤적은 정규화된 `[0,1]` 값이 아니라 **중립 대비 각도(rad)**로 저장합니다
([environment.md](environment.kr.md#궤적-데이터-형식) 참고). 정규화 방식을 쓰면 같은 값이 좌우에서
다른 각도로 매핑되어 동작이 눈에 띄게 어긋납니다 (측정값: fingertip 기준 최대 19.6 mm 차이).

---

## 2. `fix_root_link=True`를 쓰면 안 됨

USD에 이미 월드와 `palm_link`를 잇는 `global` fixed joint가 있습니다. 여기에
`ArticulationRootPropertiesCfg(fix_root_link=True)`를 주면 Isaac Lab이 fixed joint와
`ArticulationRootAPI`를 하나씩 **더** 만들어서, 초기화 시 다음 에러가 납니다.

```
RuntimeError: Expected exactly one ArticulationRootAPI prim under '/World/envs/env_0/RightHand',
found 2: ['.../Geometry/world', '.../Geometry/world/palm_link']
```

그래서 해당 플래그는 일부러 설정하지 않았습니다. 손은 USD의 `global` 조인트만으로 이미
fixed base입니다 (측정 변위 0.04 mm 미만).

---

## 3. URDF의 friction / effort 값은 그대로 쓸 수 없음

에셋의 `physxJoint:jointFriction`이 5~10, effort limit이 15 N·m로 실제 하드웨어보다 훨씬 큽니다.
그대로 두면 관절이 거의 움직이지 않습니다.

**우회 처리** — `_actuators()`에서 Isaac Lab 레퍼런스 Allegro와 같은 값으로 override합니다.

```python
ImplicitActuatorCfg(
    joint_names_expr=["joint_.*"],
    effort_limit_sim=0.5,      # USD: 15 N·m
    velocity_limit_sim=7.0,
    stiffness=3.0,
    damping=0.1,
    friction=0.01,             # USD: 5 ~ 10
)
```

USD에는 drive stiffness가 authoring되어 있지 않고 damping만 3으로 들어 있습니다.

---

## 4. 초기 관절 상태는 명시적으로 써야 함

`sim.reset()`은 USD에 authoring된 joint state(대부분 0)를 그대로 둡니다. 왼손 `joint_13_0`의
경우 그 값이 limit `[1.78, 3.14]` **밖**이라, 첫 스텝에서 손이 튀어 오릅니다.

`DirectRLEnv`는 `_reset_idx()`에서 관절 상태를 쓰므로 문제가 없지만, `run_sim.py`처럼
`SimulationContext`를 직접 쓰는 스크립트에서는 `sim.reset()` 직후 명시적으로 써야 합니다.

```python
for hand in hands.values():
    hand.write_joint_position_to_sim_index(position=hand.data.default_joint_pos.torch)
    hand.write_joint_velocity_to_sim_index(velocity=hand.data.default_joint_vel.torch)
    hand.reset()
```

---

## 5. 기타

- `allegro_hand_description_*.usda` 최상위에 Kit 세션 잔여물(`Render`, `PhysicsScene`, 카메라 prim)이
  남아 있습니다. `UsdFileCfg`는 `defaultPrim`만 reference하므로 실제 스폰에는 영향이 없습니다.
- 오른손 파일에는 최상위 `over "Physics"`에 일부 관절의 `drive:angular:physics:targetPosition`이
  남아 있습니다(예: `joint_1_0 = 80.7°`). Isaac Lab이 매 스텝 target을 덮어쓰므로 영향은 없습니다.
- Physics variant는 `physx`, `physics`, `mujoco`, `none` 네 가지가 있고 기본값은 `physx`입니다
  (`physx.usda`가 `physics.usda`를 subLayer로 포함합니다).
- 클로닝 시 `Cloning joints .../Physics/global without a body rel` 경고가 뜨는데, `global` 조인트의
  `body0`가 rigid body가 아닌 최상위 Xform이라 나오는 것입니다. 손바닥은 실제로 제자리에
  고정됩니다.
- 강체는 손당 21개(`palm_link` + 링크 16 + fingertip 4)입니다. `link_sensor_*`는 촉각 센서
  site(`IsaacSiteAPI`)라 rigid body가 아니며 `body_names`에 나오지 않습니다.
