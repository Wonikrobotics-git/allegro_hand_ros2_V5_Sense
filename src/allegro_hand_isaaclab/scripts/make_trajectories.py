# Copyright (c) 2022-2026, The Isaac Lab Project Developers (https://github.com/isaac-sim/IsaacLab/blob/main/CONTRIBUTORS.md).
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

"""Generate the demo trajectories played back by ``run_sim.py``.

Trajectories are stored as **flexion amounts in radians**: shape ``(T, 16)``, non-negative, measured
from each joint's neutral (default) position *towards* its flexed side. ``run_sim.py`` turns them
into absolute targets per hand with

    target = clamp(neutral + direction * delta, lower, upper)

where ``direction`` is the sign pointing from neutral towards whichever position limit lies further
away. Storing the magnitude rather than a normalised fraction is what keeps both hands moving
identically: the left and right assets do *not* share joint ranges (e.g. ``joint_3_0`` spans
1.86 rad on the right but 2.01 rad on the left, and the left ``joint_13_0`` only 1.35 rad), so a
normalised trajectory would map to different angles on each hand. Only the direction is mirrored,
which also absorbs the left thumb's sign flip and its ~pi offset on ``joint_13_0``
(see ``assets/allegro_hand_v5_sense.py``).

Keep :data:`MAX_FLEX` at or below the smallest range shared by both hands (~1.35 rad, set by the
left ``joint_13_0``) so playback never clamps and the hands stay symmetric.

Column order matches ``ALLEGRO_HAND_V5_SENSE_JOINT_NAMES``: ``joint_0_0`` ... ``joint_15_0``, i.e.
index 0-3, middle 4-7, ring 8-11, thumb 12-15. Within each finger, the first joint is
abduction/spread and the remaining three are flexion.

This script has no Isaac Lab dependency -- plain numpy.

Usage:
    python scripts/make_trajectories.py
"""

import os

import numpy as np

NUM_JOINTS = 16
FPS = 100.0
MAX_FLEX = 1.3
"""Largest flexion (rad) any joint is driven to. Below the tightest shared range, so no clamping."""

# per-finger joint columns, in [abduction, flex1, flex2, flex3] order
FINGERS = {
    "index": [0, 1, 2, 3],
    "middle": [4, 5, 6, 7],
    "ring": [8, 9, 10, 11],
    "thumb": [12, 13, 14, 15],
}
# fingers in the order the wave sweeps across them
WAVE_ORDER = ["index", "middle", "ring", "thumb"]

OUT_DIR = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "data")


def _smoothstep(x: np.ndarray) -> np.ndarray:
    """Hermite ease-in/ease-out on ``[0, 1]``, clamped outside."""
    x = np.clip(x, 0.0, 1.0)
    return x * x * (3.0 - 2.0 * x)


def _bump(phase: np.ndarray) -> np.ndarray:
    """Rise from 0 to 1 and back to 0 over ``phase`` in ``[0, 1]``."""
    return _smoothstep(2.0 * phase) * _smoothstep(2.0 * (1.0 - phase))


def make_wave(duration_s: float = 2.4, flex_amount: float = 0.85 * MAX_FLEX) -> np.ndarray:
    """Fingers curl one after another, index to thumb, then release."""
    num_frames = int(duration_s * FPS)
    t = np.linspace(0.0, 1.0, num_frames, endpoint=False)
    traj = np.zeros((num_frames, NUM_JOINTS), dtype=np.float32)

    # each finger owns a window of the cycle, overlapping its neighbours by half a window
    window = 1.0 / (len(WAVE_ORDER) + 1)
    for i, finger in enumerate(WAVE_ORDER):
        start = i * window
        envelope = _bump((t - start) / (2.0 * window)) * flex_amount
        _, *flex_cols = FINGERS[finger]
        for col in flex_cols:
            traj[:, col] = envelope
        if finger == "thumb":
            # the thumb also swings through its opposition joint
            traj[:, FINGERS["thumb"][0]] = envelope * 0.6
    return traj


def make_grasp(duration_s: float = 3.0, flex_amount: float = 0.95 * MAX_FLEX, hold_frac: float = 0.25) -> np.ndarray:
    """All fingers close together, hold, then open."""
    num_frames = int(duration_s * FPS)
    t = np.linspace(0.0, 1.0, num_frames, endpoint=False)
    traj = np.zeros((num_frames, NUM_JOINTS), dtype=np.float32)

    close = (1.0 - hold_frac) / 2.0
    envelope = (
        np.where(
            t < close,
            _smoothstep(t / close),
            np.where(t < close + hold_frac, 1.0, _smoothstep((1.0 - t) / close)),
        ).astype(np.float32)
        * flex_amount
    )

    for finger, cols in FINGERS.items():
        abduction, *flex_cols = cols
        for col in flex_cols:
            traj[:, col] = envelope
        if finger == "thumb":
            # thumb opposes first so it meets the fingers instead of colliding with them
            traj[:, abduction] = _smoothstep(t / (close * 0.6)).astype(np.float32) * 0.8 * MAX_FLEX
    return traj


def main():
    os.makedirs(OUT_DIR, exist_ok=True)
    for name, traj in (("wave", make_wave()), ("grasp", make_grasp())):
        path = os.path.join(OUT_DIR, f"{name}.npy")
        np.save(path, traj)
        print(f"saved {path}  shape={traj.shape}  range=[{traj.min():.3f}, {traj.max():.3f}]")


if __name__ == "__main__":
    main()
