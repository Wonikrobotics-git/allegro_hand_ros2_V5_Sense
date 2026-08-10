# Copyright (c) 2022-2026, The Isaac Lab Project Developers (https://github.com/isaac-sim/IsaacLab/blob/main/CONTRIBUTORS.md).
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

"""Allegro Hand V5 sense simulation demo.

Spawns the hands in Isaac Lab and plays a recorded joint trajectory in a loop. This is the
zero-setup entry point: it does not need the extension package installed and does not go through
Gym or the RL environment. For the trainable environment see ``scripts/random_agent.py`` and
``scripts/skrl/train.py``.

Usage:
    python run_sim.py                          # both hands, wave trajectory
    python run_sim.py --side right             # right hand only
    python run_sim.py --traj grasp             # close / hold / open
    python run_sim.py --headless --max_steps 500
"""

import argparse
import os
import sys

from isaaclab.app import AppLauncher

# make the extension package importable straight from the source tree, so the demo runs without
# 'pip install -e source/v5_sense_isaaclab_pipeline'
_REPO_DIR = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(_REPO_DIR, "source", "v5_sense_isaaclab_pipeline"))

parser = argparse.ArgumentParser(description="Allegro Hand V5 sense simulation demo.")
parser.add_argument("--side", choices=["left", "right", "both"], default="both", help="Which hand(s) to spawn.")
parser.add_argument("--traj", default="wave", help="Trajectory name under 'data/' (without .npy), or a file path.")
parser.add_argument("--speed", type=float, default=1.0, help="Trajectory playback speed multiplier.")
parser.add_argument(
    "--max_steps", type=int, default=None, help="Stop after this many steps. Defaults to None (run until closed)."
)
AppLauncher.add_app_launcher_args(parser)
# the demo is meant to be watched, so open the Kit viewer unless told otherwise
parser.set_defaults(visualizer=["kit"])
args_cli = parser.parse_args()

app_launcher = AppLauncher(args_cli)
simulation_app = app_launcher.app

"""Rest everything follows."""

import numpy as np
import torch
from v5_sense_isaaclab_pipeline.assets import (
    ALLEGRO_HAND_V5_SENSE_LEFT_CFG,
    ALLEGRO_HAND_V5_SENSE_RIGHT_CFG,
)
from v5_sense_isaaclab_pipeline.assets.allegro_hand_v5_sense import ALLEGRO_HAND_V5_SENSE_JOINT_NAMES

import isaaclab.sim as sim_utils
from isaaclab.assets import Articulation
from isaaclab.sim import SimulationCfg, SimulationContext

TRAJECTORY_FPS = 100.0
"""Rate the trajectories in 'data/' were authored at. The sim runs at the same rate."""


def load_trajectory(name: str) -> np.ndarray:
    """Load a ``(T, 16)`` flexion trajectory (radians from neutral) by name or path."""
    path = name if os.path.isfile(name) else os.path.join(_REPO_DIR, "data", f"{name}.npy")
    if not os.path.isfile(path):
        available = sorted(f[:-4] for f in os.listdir(os.path.join(_REPO_DIR, "data")) if f.endswith(".npy"))
        raise FileNotFoundError(f"No trajectory '{name}'. Available in 'data/': {available}")
    traj = np.load(path)
    if traj.ndim != 2 or traj.shape[1] != len(ALLEGRO_HAND_V5_SENSE_JOINT_NAMES):
        raise ValueError(f"Expected a (T, {len(ALLEGRO_HAND_V5_SENSE_JOINT_NAMES)}) trajectory, got {traj.shape}.")
    return traj


def flex_frame(hand: Articulation, joint_ids: list[int]) -> tuple[torch.Tensor, torch.Tensor, torch.Tensor]:
    """Return ``(neutral, direction, limits)`` used to turn a trajectory into joint targets.

    ``direction`` is ``+1`` / ``-1`` pointing from the neutral pose towards whichever position limit
    lies further away, i.e. the way the joint curls. Applying only the *sign* per hand -- and taking
    the flexion magnitude straight from the trajectory -- is what keeps both hands moving
    identically, since the left and right assets do not share joint ranges.
    """
    neutral = hand.data.default_joint_pos.torch[:, joint_ids]
    limits = hand.data.soft_joint_pos_limits.torch[:, joint_ids]
    lower, upper = limits[..., 0], limits[..., 1]
    direction = torch.where((upper - neutral).abs() >= (neutral - lower).abs(), 1.0, -1.0)
    return neutral, direction, limits


def main():
    """Spawn the hand(s) and loop the trajectory."""
    sim = SimulationContext(SimulationCfg(dt=1.0 / TRAJECTORY_FPS, device=args_cli.device))
    # viewed from -X so we look at the palms rather than the backs of the hands
    sim.set_camera_view([-0.85, -0.30, 0.40], [0.0, 0.0, 0.22])

    # ground plane and light
    ground_cfg = sim_utils.GroundPlaneCfg()
    ground_cfg.func("/World/ground", ground_cfg)
    light_cfg = sim_utils.DomeLightCfg(intensity=2500.0, color=(0.9, 0.9, 0.9))
    light_cfg.func("/World/light", light_cfg)

    # hands -- a single hand is centred, both hands keep their side-by-side offsets
    wanted = ["right", "left"] if args_cli.side == "both" else [args_cli.side]
    source_cfgs = {"right": ALLEGRO_HAND_V5_SENSE_RIGHT_CFG, "left": ALLEGRO_HAND_V5_SENSE_LEFT_CFG}
    hands = {}
    for side in wanted:
        cfg = source_cfgs[side].replace(prim_path=f"/World/{side.capitalize()}Hand")
        if args_cli.side != "both":
            cfg.init_state.pos = (0.0, 0.0, cfg.init_state.pos[2])
        hands[side] = Articulation(cfg)

    sim.reset()

    # Put the joints in their configured neutral pose. Without this they keep whatever state the USD
    # was authored with, which for the left thumb's 'joint_13_0' is 0.0 -- outside its [1.78, 3.14]
    # range -- so the hand would snap on the first step.
    for hand in hands.values():
        hand.write_joint_position_to_sim_index(position=hand.data.default_joint_pos.torch)
        hand.write_joint_velocity_to_sim_index(velocity=hand.data.default_joint_vel.torch)
        hand.reset()

    # resolve joints in the documented order, and precompute the neutral pose and flexion direction
    joint_ids, neutral, direction, limits = {}, {}, {}, {}
    for side, hand in hands.items():
        ids, names = hand.find_joints(ALLEGRO_HAND_V5_SENSE_JOINT_NAMES, preserve_order=True)
        joint_ids[side] = ids
        neutral[side], direction[side], limits[side] = flex_frame(hand, ids)
        print(f"[INFO]: {side} hand -- {len(names)} actuated joints: {names}")

    traj = torch.from_numpy(load_trajectory(args_cli.traj)).to(device=sim.device, dtype=torch.float32)
    print(f"[INFO]: Playing '{args_cli.traj}' -- {len(traj)} frames at {TRAJECTORY_FPS:g} fps, speed x{args_cli.speed}")

    step = 0
    while simulation_app.is_running():
        if args_cli.max_steps is not None and step >= args_cli.max_steps:
            print(f"[INFO]: Reached the requested {args_cli.max_steps} steps. Exiting.")
            break

        frame = traj[int(step * args_cli.speed) % len(traj)]
        for side, hand in hands.items():
            target = neutral[side] + direction[side] * frame
            target = target.clamp(limits[side][..., 0], limits[side][..., 1])
            hand.set_joint_position_target_index(target=target, joint_ids=joint_ids[side])
            hand.write_data_to_sim()

        sim.step()
        for hand in hands.values():
            hand.update(sim.get_physics_dt())
        step += 1


if __name__ == "__main__":
    main()
    simulation_app.close()
