# Asset Notes

**English** | [한국어](asset_notes.kr.md) — [← README](../README.md)

Quirks introduced by the URDF → USD conversion. Each has a workaround in the code; read this before
re-exporting the assets.

---

## 1. The left thumb's `joint_13_0` is not mirrored

Every other joint on the left hand corresponds to the right, but the thumb rotation joint
`joint_13_0` is authored with a range of `[1.780, 3.140] rad` (`[102°, 180°]`) where the mirrored
value would be `[-1.780, 0.260]`. The cause is `link_12_0`'s joint frame being rotated by ~π relative
to the mirrored frame (its `localRot0` differs from the right hand's by roughly 180°).

As a result this joint's neutral value is **about π**, not `0.0`. Leaving it at `0.0` makes Isaac Lab
reject the articulation outright with `"default positions out of the limits"`.

**Workaround** — in `assets/allegro_hand_v5_sense.py`:

```python
LEFT_THUMB_ROT_JOINT_NEUTRAL = 3.13
...
joint_pos={"^(?!joint_13_0$).*": 0.0, "joint_13_0": LEFT_THUMB_ROT_JOINT_NEUTRAL},
```

Set it back to `0.0` if the left URDF is ever re-exported with a properly mirrored thumb frame.

### Knock-on effect: mismatched joint ranges

Because of this, the left `joint_13_0` spans only 1.35 rad against the right hand's 1.78 rad. Other
joints differ slightly too.

| Joint | Right range | Left range |
|---|---|---|
| `joint_0_0` | `[-0.700, +0.262]` | `[-0.300, +0.300]` |
| `joint_3_0` / `7_0` / `11_0` | `[-0.100, +1.860]` | `[-0.100, +2.010]` |
| `joint_13_0` | `[-0.260, +1.780]` | `[+1.780, +3.140]` |
| `joint_14_0` | `[-0.050, +1.850]` | `[-1.780, +0.260]` |

This is why the demo trajectories store **angles in radians from neutral** rather than a normalised
`[0, 1]` fraction (see [environment.md](environment.md#trajectory-data-format)). With normalised
values the same number maps to different angles on each hand and the motion visibly diverges —
measured at up to 19.6 mm of fingertip asymmetry.

---

## 2. Do not set `fix_root_link=True`

The USD already contains a `global` fixed joint between the world and `palm_link`. Passing
`ArticulationRootPropertiesCfg(fix_root_link=True)` makes Isaac Lab author a *second* fixed joint and
a second `ArticulationRootAPI`, which fails at initialization:

```
RuntimeError: Expected exactly one ArticulationRootAPI prim under '/World/envs/env_0/RightHand',
found 2: ['.../Geometry/world', '.../Geometry/world/palm_link']
```

The flag is deliberately left unset. The USD's own `global` joint already makes the hand fixed-base
(measured drift under 0.04 mm).

---

## 3. The URDF's friction and effort values are unusable

The asset carries `physxJoint:jointFriction` values of 5–10 and an effort limit of 15 N·m, far above
the real hardware. Left as-is, the joints barely move.

**Workaround** — `_actuators()` overrides them with the same values as Isaac Lab's reference Allegro:

```python
ImplicitActuatorCfg(
    joint_names_expr=["joint_.*"],
    effort_limit_sim=0.5,      # USD: 15 N·m
    velocity_limit_sim=7.0,
    stiffness=3.0,
    damping=0.1,
    friction=0.01,             # USD: 5 – 10
)
```

The USD authors no drive stiffness at all, only a damping of 3.

---

## 4. Initial joint state must be written explicitly

`sim.reset()` leaves the joint state authored in the USD (mostly `0`) in place. For the left
`joint_13_0` that value sits **outside** its `[1.78, 3.14]` limits, so the hand snaps on the first
step.

`DirectRLEnv` writes joint state in `_reset_idx()`, so the training environment is unaffected. A
script driving `SimulationContext` directly — like `run_sim.py` — has to do it right after
`sim.reset()`:

```python
for hand in hands.values():
    hand.write_joint_position_to_sim_index(position=hand.data.default_joint_pos.torch)
    hand.write_joint_velocity_to_sim_index(velocity=hand.data.default_joint_vel.torch)
    hand.reset()
```

---

## 5. Miscellaneous

- The top level of `allegro_hand_description_*.usda` still holds Kit session leftovers (`Render`,
  `PhysicsScene`, camera prims). `UsdFileCfg` only references the `defaultPrim`, so they do not
  affect spawning.
- The right hand file has leftover `drive:angular:physics:targetPosition` values on some joints in
  its top-level `over "Physics"` (e.g. `joint_1_0 = 80.7°`). Isaac Lab overwrites targets every step,
  so they have no effect.
- Four Physics variants exist — `physx`, `physics`, `mujoco`, `none` — with `physx` as the default
  (`physx.usda` sublayers `physics.usda`).
- Cloning emits `Cloning joints .../Physics/global without a body rel` warnings because the `global`
  joint's `body0` is the top-level Xform rather than a rigid body. The palms do stay anchored.
- Each hand has 21 rigid bodies (`palm_link` + 16 links + 4 fingertips). `link_sensor_*` prims are
  tactile sensor sites (`IsaacSiteAPI`), not rigid bodies, and do not appear in `body_names`.
