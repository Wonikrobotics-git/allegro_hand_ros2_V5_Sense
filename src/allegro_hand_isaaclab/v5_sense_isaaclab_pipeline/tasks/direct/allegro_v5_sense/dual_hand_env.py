# Copyright (c) 2022-2026, The Isaac Lab Project Developers (https://github.com/isaac-sim/IsaacLab/blob/main/CONTRIBUTORS.md).
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

"""Direct RL environment spawning the left and right Allegro Hand V5 sense hands."""

from __future__ import annotations

from collections.abc import Sequence
from typing import TYPE_CHECKING

import torch

import isaaclab.sim as sim_utils
from isaaclab.assets import Articulation
from isaaclab.envs import DirectRLEnv
from isaaclab.sim.spawners.from_files import GroundPlaneCfg, spawn_ground_plane
from isaaclab.utils.math import sample_uniform

if TYPE_CHECKING:
    from .dual_hand_env_cfg import V5SenseDualHandEnvCfg


class V5SenseDualHandEnv(DirectRLEnv):
    """Two fixed-base Allegro Hand V5 sense hands driven by joint position targets.

    Actions are ``[right_hand (16), left_hand (16)]`` in ``[-1, 1]`` and are mapped onto each
    joint's position limits. Observations are ``[right_pos, right_vel, left_pos, left_vel]``.
    The reward is zero and the only termination is the episode time limit: downstream pipelines
    are expected to override :meth:`_get_rewards` / :meth:`_get_dones` with their actual task.
    """

    cfg: V5SenseDualHandEnvCfg

    def __init__(self, cfg: V5SenseDualHandEnvCfg, render_mode: str | None = None, **kwargs):
        super().__init__(cfg, render_mode, **kwargs)

        # resolve the actuated joints in the order given by the config (not the physics ordering)
        self.right_joint_ids, self.right_joint_names = self.right_hand.find_joints(
            self.cfg.actuated_joint_names, preserve_order=True
        )
        self.left_joint_ids, self.left_joint_names = self.left_hand.find_joints(
            self.cfg.actuated_joint_names, preserve_order=True
        )
        self.num_hand_dofs = len(self.right_joint_ids)
        print(f"[INFO]: Right hand actuated joints ({self.num_hand_dofs}): {self.right_joint_names}")
        print(f"[INFO]: Left hand actuated joints ({len(self.left_joint_ids)}): {self.left_joint_names}")

        # joint position limits of the actuated joints, shape (num_envs, num_hand_dofs)
        right_limits = self.right_hand.data.soft_joint_pos_limits.torch[:, self.right_joint_ids]
        left_limits = self.left_hand.data.soft_joint_pos_limits.torch[:, self.left_joint_ids]
        self.joint_pos_lower = torch.cat((right_limits[..., 0], left_limits[..., 0]), dim=-1)
        self.joint_pos_upper = torch.cat((right_limits[..., 1], left_limits[..., 1]), dim=-1)

        # default joint positions of the actuated joints, shape (num_envs, 2 * num_hand_dofs)
        self.default_joint_pos = torch.cat(
            (
                self.right_hand.data.default_joint_pos.torch[:, self.right_joint_ids],
                self.left_hand.data.default_joint_pos.torch[:, self.left_joint_ids],
            ),
            dim=-1,
        ).clone()

        # buffers
        self.actions = torch.zeros((self.num_envs, self.cfg.action_space), device=self.device)
        self.joint_pos_target = self.default_joint_pos.clone()

    """
    Scene setup.
    """

    def _setup_scene(self):
        self.right_hand = Articulation(self.cfg.right_hand_cfg)
        self.left_hand = Articulation(self.cfg.left_hand_cfg)
        # add ground plane
        spawn_ground_plane(prim_path="/World/ground", cfg=GroundPlaneCfg())
        # clone and replicate
        self.scene.clone_environments(copy_from_source=False)
        # we need to explicitly filter collisions for CPU simulation
        if self.device == "cpu":
            self.scene.filter_collisions(global_prim_paths=[])
        # add articulations to scene
        self.scene.articulations["right_hand"] = self.right_hand
        self.scene.articulations["left_hand"] = self.left_hand
        # add lights
        light_cfg = sim_utils.DomeLightCfg(intensity=2000.0, color=(0.75, 0.75, 0.75))
        light_cfg.func("/World/Light", light_cfg)

    """
    Stepping.
    """

    def _pre_physics_step(self, actions: torch.Tensor) -> None:
        self.actions = actions.clone().clamp(-1.0, 1.0)
        # map [-1, 1] onto the joint range, then smooth the targets to avoid jerky motion
        target = scale(self.cfg.action_scale * self.actions, self.joint_pos_lower, self.joint_pos_upper)
        self.joint_pos_target = (
            self.cfg.act_moving_average * target + (1.0 - self.cfg.act_moving_average) * self.joint_pos_target
        )
        self.joint_pos_target = self.joint_pos_target.clamp(self.joint_pos_lower, self.joint_pos_upper)

    def _apply_action(self) -> None:
        self.right_hand.set_joint_position_target_index(
            target=self.joint_pos_target[:, : self.num_hand_dofs], joint_ids=self.right_joint_ids
        )
        self.left_hand.set_joint_position_target_index(
            target=self.joint_pos_target[:, self.num_hand_dofs :], joint_ids=self.left_joint_ids
        )

    """
    MDP.
    """

    def _get_observations(self) -> dict:
        obs = torch.cat(
            (
                self.right_hand.data.joint_pos.torch[:, self.right_joint_ids],
                self.cfg.vel_obs_scale * self.right_hand.data.joint_vel.torch[:, self.right_joint_ids],
                self.left_hand.data.joint_pos.torch[:, self.left_joint_ids],
                self.cfg.vel_obs_scale * self.left_hand.data.joint_vel.torch[:, self.left_joint_ids],
            ),
            dim=-1,
        )
        return {"policy": obs}

    def _get_rewards(self) -> torch.Tensor:
        # no task is defined yet -- downstream pipelines plug their own reward in here
        return torch.zeros(self.num_envs, device=self.device)

    def _get_dones(self) -> tuple[torch.Tensor, torch.Tensor]:
        time_out = self.episode_length_buf >= self.max_episode_length - 1
        terminated = torch.zeros_like(time_out)
        return terminated, time_out

    def _reset_idx(self, env_ids: Sequence[int] | None):
        if env_ids is None:
            env_ids = self.right_hand._ALL_INDICES
        super()._reset_idx(env_ids)

        noise = sample_uniform(
            -self.cfg.reset_joint_pos_noise,
            self.cfg.reset_joint_pos_noise,
            (len(env_ids), self.cfg.action_space),
            self.device,
        )
        joint_pos = (self.default_joint_pos[env_ids] + noise).clamp(
            self.joint_pos_lower[env_ids], self.joint_pos_upper[env_ids]
        )
        joint_vel = torch.zeros_like(joint_pos)
        self.joint_pos_target[env_ids] = joint_pos

        for hand, joint_ids, hand_slice in (
            (self.right_hand, self.right_joint_ids, slice(None, self.num_hand_dofs)),
            (self.left_hand, self.left_joint_ids, slice(self.num_hand_dofs, None)),
        ):
            hand.write_joint_position_to_sim_index(
                position=joint_pos[:, hand_slice], joint_ids=joint_ids, env_ids=env_ids
            )
            hand.write_joint_velocity_to_sim_index(
                velocity=joint_vel[:, hand_slice], joint_ids=joint_ids, env_ids=env_ids
            )
            hand.set_joint_position_target_index(target=joint_pos[:, hand_slice], joint_ids=joint_ids, env_ids=env_ids)


@torch.jit.script
def scale(x: torch.Tensor, lower: torch.Tensor, upper: torch.Tensor) -> torch.Tensor:
    """Map ``x`` from ``[-1, 1]`` onto ``[lower, upper]``."""
    return 0.5 * (x + 1.0) * (upper - lower) + lower
