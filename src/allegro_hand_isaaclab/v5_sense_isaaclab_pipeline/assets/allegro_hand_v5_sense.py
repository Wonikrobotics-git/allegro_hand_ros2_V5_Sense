# Copyright (c) 2022-2026, The Isaac Lab Project Developers (https://github.com/isaac-sim/IsaacLab/blob/main/CONTRIBUTORS.md).
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

"""Configuration for the Allegro Hand V5 "sense" (tactile) left/right hands from Wonik Robotics.

The following configurations are available:

* :obj:`ALLEGRO_HAND_V5_SENSE_RIGHT_CFG`: right hand, implicit actuator model.
* :obj:`ALLEGRO_HAND_V5_SENSE_LEFT_CFG`: left hand, implicit actuator model.

Both hands are converted from URDF and share the same joint/link naming scheme:

* 16 revolute (actuated) joints: ``joint_0_0`` ... ``joint_15_0``.
* 4 fixed fingertip joints (``joint_*_0_tip``) and a ``global`` fixed joint pinning ``palm_link``
  to the world, i.e. the articulation has a fixed base.
* ``link_sensor_*`` prims are tactile sensor sites (``IsaacSiteAPI``), not rigid bodies.

The articulation root (``PhysicsArticulationRootAPI``) sits on ``Geometry/world/palm_link``; Isaac Lab
locates it automatically, so :attr:`ArticulationCfg.articulation_root_prim_path` is left unset.

In the asset's own frame the four fingers extend along ``-Z``, i.e. the hand hangs fingers-down under
the identity orientation. Both configurations below therefore apply :data:`UPRIGHT_ROT`, a 180-degree
rotation about ``Y``, so the fingers point up. That axis is chosen over ``X`` because it leaves the
thumb direction untouched (``-Y`` on the right hand, ``+Y`` on the left), keeping the two hands
mirrored about the ``XZ`` plane. Each hand spans about 0.20 m across and 0.22 m along the fingers.
"""

import os

import isaaclab.sim as sim_utils
from isaaclab.actuators import ImplicitActuatorCfg
from isaaclab.assets.articulation import ArticulationCfg

from . import V5_SENSE_ASSET_DIR

##
# Constants
##

UPRIGHT_ROT = (0.0, 0.0, 1.0, 0.0)
"""Spawn orientation (w, x, y, z): 180 degrees about ``Y``, standing the hand fingers-up.

The URDF-converted asset points its fingers along ``-Z``. Rotating about ``Y`` flips that to ``+Z``
while preserving the thumb's ``+/-Y`` direction, so the left and right hands stay mirrored. Use the
identity ``(1, 0, 0, 0)`` instead if you want the original fingers-down pose, e.g. for a hand mounted
on the end of an arm.
"""

PALM_HEIGHT = 0.2
"""Height (m) of ``palm_link`` above the ground. Puts the upright hand's lowest point at ~0.12 m."""

LEFT_THUMB_ROT_JOINT_NEUTRAL = 3.13
"""Neutral position (rad) of the left hand's thumb rotation joint ``joint_13_0``.

The left asset mirrors the right one everywhere except here: ``link_12_0``'s joint frame is rotated
by ~pi relative to the mirrored right frame, so ``joint_13_0`` is authored with a range of
``[1.780, 3.140] rad`` (``[102, 180] deg``) instead of the mirrored ``[-1.780, 0.260] rad``. Its
neutral pose is therefore at the *upper* limit (~pi), not at 0.0. Using 0.0 makes Isaac Lab reject
the articulation with "default positions out of the limits". Set this back to ``0.0`` if the left
URDF is ever re-exported with a properly mirrored thumb frame.
"""

ALLEGRO_HAND_V5_SENSE_JOINT_NAMES = [f"joint_{i}_0" for i in range(16)]
"""Actuated joint names, in URDF/index order (``joint_0_0`` ... ``joint_15_0``).

Index finger is ``0-3``, middle ``4-7``, ring ``8-11``, thumb ``12-15``. Use this list together with
:meth:`~isaaclab.assets.Articulation.find_joints` (``preserve_order=True``) to get a deterministic
action/observation layout that does not depend on the physics backend's joint ordering.
"""


def _usd_path(hand: str) -> str:
    """Return the absolute path to the hand USD file and verify it exists."""
    path = os.path.join(V5_SENSE_ASSET_DIR, f"v5_sense_{hand}", f"allegro_hand_description_{hand}.usda")
    if not os.path.isfile(path):
        raise FileNotFoundError(
            f"Could not find the Allegro Hand V5 sense ({hand}) USD file at: '{path}'."
            " Set the 'V5_SENSE_ASSET_DIR' environment variable to the directory holding the"
            " 'v5_sense_left/' and 'v5_sense_right/' asset folders."
        )
    return path


##
# Shared spawn/actuator settings
##


def _spawn(hand: str) -> sim_utils.UsdFileCfg:
    """Spawn configuration for one hand."""
    return sim_utils.UsdFileCfg(
        usd_path=_usd_path(hand),
        activate_contact_sensors=False,
        rigid_props=sim_utils.RigidBodyPropertiesCfg(
            disable_gravity=False,
            retain_accelerations=False,
            enable_gyroscopic_forces=False,
            angular_damping=0.01,
            max_linear_velocity=1000.0,
            max_angular_velocity=3600.0,
            max_depenetration_velocity=1000.0,
            max_contact_impulse=1e32,
        ),
        articulation_props=sim_utils.ArticulationRootPropertiesCfg(
            # NOTE: 'fix_root_link' is deliberately left unset. The asset already contains a
            # 'global' fixed joint between the world and 'palm_link', and setting the flag makes
            # Isaac Lab author a second fixed joint plus a second ArticulationRootAPI prim, which
            # then fails the "exactly one articulation root" check at initialization.
            enabled_self_collisions=False,
            solver_position_iteration_count=8,
            solver_velocity_iteration_count=0,
            sleep_threshold=0.005,
            stabilization_threshold=0.0005,
        ),
    )


def _actuators() -> dict[str, ImplicitActuatorCfg]:
    """PD actuators for the 16 finger joints.

    The URDF-converted asset carries ``physxJoint:jointFriction`` values of 5-10 and an effort limit
    of 15 N-m, which are far above the real hardware and would leave the joints immobile. The values
    below override them and match the Isaac Lab reference Allegro Hand configuration.
    """
    return {
        "fingers": ImplicitActuatorCfg(
            joint_names_expr=["joint_.*"],
            effort_limit_sim=0.5,
            velocity_limit_sim=7.0,
            stiffness=3.0,
            damping=0.1,
            friction=0.01,
        ),
    }


##
# Configuration
##

ALLEGRO_HAND_V5_SENSE_RIGHT_CFG = ArticulationCfg(
    spawn=_spawn("right"),
    init_state=ArticulationCfg.InitialStateCfg(
        pos=(0.0, -0.15, PALM_HEIGHT),
        rot=UPRIGHT_ROT,
        joint_pos={".*": 0.0},
        joint_vel={".*": 0.0},
    ),
    actuators=_actuators(),
    soft_joint_pos_limit_factor=1.0,
)
"""Configuration of the right Allegro Hand V5 sense: fingers up, thumb towards -Y."""

ALLEGRO_HAND_V5_SENSE_LEFT_CFG = ArticulationCfg(
    spawn=_spawn("left"),
    init_state=ArticulationCfg.InitialStateCfg(
        pos=(0.0, 0.15, PALM_HEIGHT),
        rot=UPRIGHT_ROT,
        # the negative lookahead keeps 'joint_13_0' from matching both patterns
        joint_pos={"^(?!joint_13_0$).*": 0.0, "joint_13_0": LEFT_THUMB_ROT_JOINT_NEUTRAL},
        joint_vel={".*": 0.0},
    ),
    actuators=_actuators(),
    soft_joint_pos_limit_factor=1.0,
)
"""Configuration of the left Allegro Hand V5 sense: fingers up, thumb towards +Y."""
