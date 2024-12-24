#!/usr/bin/env python
# -*-coding:utf-8 -*-
"""
@file run_hilti_multi_inertial.py
@author Yanwei Du (duyanwei0702@gmail.com)
@date 11-30-2024
@version 1.0
@license Copyright (c) 2024
@desc None
"""

import subprocess
from pathlib import Path


DATA_ROOT = Path("/mnt/DATA/datasets/hilti/2022/euroc_format/")
RESULT_ROOT = Path("/mnt/DATA/experiments/multi_cam/OpenMAVIS/disturbed_extrinsics")
NODE_ROOT = Path("/home/yanwei/slam_ws/src/opensource/multi/OpenMAVIS/")
ROUNDS = 5

MODES = ["t005r005_tx10"]


for mode in MODES:
    exp_dir = RESULT_ROOT / mode
    exp_dir.mkdir(exist_ok=True, parents=True)
    for round_idx in range(ROUNDS):
        node = NODE_ROOT / "Examples/Multi-Inertial/multi_inertial_euroc"
        voc_filename = NODE_ROOT / "Vocabulary/ORBvoc.txt"
        yaml_filename = NODE_ROOT / "Examples/Multi-Inertial/HiltiChallenge2022.yaml"
        seq_prefix = DATA_ROOT / "exp04_construction_upper_level"
        timestamp_filename = seq_prefix / "mav0/timestamps.txt"

        output_prefix = exp_dir / f"Round{round_idx+1}"

        cmd = f"{node} {voc_filename} {yaml_filename} {seq_prefix} {timestamp_filename} {output_prefix}"

        print(cmd)
        subprocess.call(cmd, shell=True)
