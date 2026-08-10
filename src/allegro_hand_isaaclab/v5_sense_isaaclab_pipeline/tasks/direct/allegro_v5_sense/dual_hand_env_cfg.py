# Copyright (c) 2022-2026, The Isaac Lab Project Developers (https://github.com/isaac-sim/IsaacLab/blob/main/CONTRIBUTORS.md).
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

"""Configuration for the Allegro Hand V5 sense dual-hand scene."""

from isaaclab.assets import ArticulationCfg
from isaaclab.envs import DirectRLEnvCfg
from isaaclab.scene import InteractiveSceneCfg
from isaaclab.sim import SimulationCfg
from isaaclab.sim.spawners.materials.physics_materials_cfg import RigidBodyMaterialCfg
from isaaclab.utils.configclass import configclass

from v5_sense_isaaclab_pipeline.assets.allegro_hand_v5_sense import (
    ALLEGRO_HAND_V5_SENSE_JOINT_NAMES,
    ALLEGRO_HAND_V5_SENSE_LEFT_CFG,
    ALLEGRO_HAND_V5_SENSE_RIGHT_CFG,
)

NUM_HAND_DOFS = len(ALLEGRO_HAND_V5_SENSE_JOINT_NAMES)
"""Number of actuated joints per hand (16)."""


@configclass
class V5SenseDualHandEnvCfg(DirectRLEnvCfg):
    """Both Allegro Hand V5 sense hands spawned side by side.

    This environment is intentionally task-free: the reward is zero and episodes only end on the
    time limit. It exists to validate that both hands load, articulate and step correctly, and to
    serve as the scene definition that a downstream training pipeline builds its task on top of.
    """

    # env
    decimation = 4
    episode_length_s = 10.0

    # - spaces definition
    action_space = 2 * NUM_HAND_DOFS  # 32: right hand joints, then left hand joints
    observation_space = 4 * NUM_HAND_DOFS  # 64: [right pos, right vel, left pos, left vel]
    state_space = 0

    # simulation
    sim: SimulationCfg = SimulationCfg(
        dt=1 / 120,
        render_interval=decimation,
        physics_material=RigidBodyMaterialCfg(static_friction=1.0, dynamic_friction=1.0),
    )

    # robots
    right_hand_cfg: ArticulationCfg = ALLEGRO_HAND_V5_SENSE_RIGHT_CFG.replace(prim_path="/World/envs/env_.*/RightHand")
    left_hand_cfg: ArticulationCfg = ALLEGRO_HAND_V5_SENSE_LEFT_CFG.replace(prim_path="/World/envs/env_.*/LeftHand")

    # scene
    scene: InteractiveSceneCfg = InteractiveSceneCfg(num_envs=64, env_spacing=1.0, replicate_physics=True)

    # custom parameters/scales
    # - actuated joints, resolved in this exact order so the action/observation layout is stable
    actuated_joint_names: list[str] = ALLEGRO_HAND_V5_SENSE_JOINT_NAMES
    # - actions in [-1, 1] are mapped onto this fraction of each joint's range around its default
    action_scale = 1.0
    # - exponential moving average applied to the position targets (1.0 disables smoothing)
    act_moving_average = 0.3
    # - observation scaling
    vel_obs_scale = 0.2
    # - uniform noise (in rad) added to the default joint positions on reset
    reset_joint_pos_noise = 0.05
