#!/usr/bin/env python
# -*-coding:utf-8 -*-
"""
@file eval.py
@author Yanwei Du (duyanwei0702@gmail.com)
@date 12-22-2024
@version 1.0
@license Copyright (c) 2024
@desc None
"""

import numpy as np
from evo.core.trajectory import PoseTrajectory3D
from evo.tools import file_interface
from evo.core import metrics
from evo.core import sync
from evo.main_ape import ape as evo_ape

from matplotlib import pyplot as plt

from pathlib import Path


def load_trajectory(file_path):
    # Load the trajectory from a TUM format file
    return file_interface.read_tum_trajectory_file(file_path)


def compute_absolute_errors(estimate, ground_truth):
    estimate.align(ground_truth)
    result = evo_ape(ground_truth, estimate, pose_relation=metrics.PoseRelation.translation_part)
    return result


DATA_ROOT = Path("/mnt/DATA/datasets/hilti/2022/euroc_format/")
RESULT_ROOT = Path("/mnt/DATA/experiments/multi_cam/OpenMAVIS/disturbed_extrinsics")
NODE_ROOT = Path("/home/yanwei/slam_ws/src/opensource/multi/OpenMAVIS/")
ROUNDS = 5

MODES = ["t005r005_tx10", "t100r100", "t050r050", "t010r010", "t005r005", "t000r000"]
MODES.reverse()


# Load ground truth
ground_truth = load_trajectory(DATA_ROOT / "../exp_04_construction_upper_level_imu.txt")

# Variables to store metrics
num_rounds = 5
rmse_table = []

# Process each round
for mode_name in MODES:
    ape_results = []
    for i in range(1, num_rounds + 1):
        estimated_path = RESULT_ROOT / mode_name / f"Round{i}_AllFrameTrajectory.txt"
        estimated_traj = load_trajectory(estimated_path)

        # Synchronize estimated trajectory with the ground truth
        max_diff = 0.01  # max timestamp difference for synchronization (in seconds)
        traj_est_sync, traj_gt_sync = sync.associate_trajectories(estimated_traj, ground_truth, max_diff)

        # Compute APE (Absolute Pose Error)
        ape = compute_absolute_errors(traj_est_sync, traj_gt_sync)
        ape_results.append(ape)

    # Calculate mean APE across all rounds
    rmse = [ape.stats["rmse"] for ape in ape_results]
    rmse.append(np.nanmean(rmse))
    rmse_table.append(rmse)
np.savetxt(RESULT_ROOT / "result.txt", rmse_table, fmt="%.4f", header=f"Round 1 ... {num_rounds} Mean")

fig = plt.figure()
plt.boxplot(rmse_table, showmeans=True)
plt.xticks(range(1, ROUNDS + 2), MODES)
plt.ylabel("RMSE (m)")
plt.title("MAVIS Hilti 2022 Exp04")
plt.tight_layout()
plt.show()
