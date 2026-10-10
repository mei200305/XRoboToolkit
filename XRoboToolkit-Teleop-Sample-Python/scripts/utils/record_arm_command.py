#!/usr/bin/env python3
"""
Record /throw/arm_joint_command position data for jitter analysis.

Subscribes to /throw/arm_joint_command and logs:
  - Timestamp (header.stamp)
  - 14 joint positions (left arm 7 + right arm 7)
  - Frame-by-frame delta for each joint

Outputs:
  - CSV file: timestamped joint positions + deltas
  - Console: real-time summary (max delta per frame)

Usage:
  python scripts/utils/record_arm_command.py
  python scripts/utils/record_arm_command.py --duration 30
  python scripts/utils/record_arm_command.py --output /tmp/jitter_analysis.csv
"""

import csv
import os
import signal
import sys
import time
from datetime import datetime

import numpy as np
from sensor_msgs.msg import JointState

from xrobotoolkit_teleop.hardware.interface import ros2_compat as ros2


class ArmCommandRecorder:
    def __init__(self, node, output_path: str, duration: float = 0):
        self.node = node
        self.output_path = output_path
        self.duration = duration  # 0 = run until Ctrl-C
        self.start_time = time.time()

        self.prev_positions = None
        self.frame_count = 0
        self.max_deltas = []  # per-frame max abs delta

        self.csv_file = open(output_path, "w", newline="")
        self.writer = csv.writer(self.csv_file)

        # Header
        joint_names = [
            "arml_1_joint", "arml_2_joint", "arml_3_joint", "arml_4_joint",
            "arml_5_joint", "arml_6_joint", "arml_7_joint",
            "armr_1_joint", "armr_2_joint", "armr_3_joint", "armr_4_joint",
            "armr_5_joint", "armr_6_joint", "armr_7_joint",
        ]
        header = ["secs", "nsecs"]
        for name in joint_names:
            header.append(f"{name}_pos")
        for name in joint_names:
            header.append(f"{name}_delta")
        self.num_joints = len(joint_names)
        self.writer.writerow(header)

        self.sub = self.node.create_subscription(
            JointState, "/throw/arm_joint_command", self._callback, 1000
        )
        print(f"Recording /throw/arm_joint_command → {output_path}")
        print(f"Duration: {'unlimited (Ctrl-C to stop)' if duration == 0 else f'{duration}s'}")
        print()

    def _callback(self, msg: JointState):
        if len(msg.position) < self.num_joints:
            return

        self.frame_count += 1
        positions = np.array(msg.position[:self.num_joints], dtype=float)

        # Compute frame-to-frame deltas
        if self.prev_positions is not None:
            deltas = positions - self.prev_positions
        else:
            deltas = np.zeros(self.num_joints)

        # Write row (ROS 2 stamp uses sec/nanosec)
        row = [msg.header.stamp.sec, msg.header.stamp.nanosec]
        row.extend([f"{p:.8f}" for p in positions])
        row.extend([f"{d:.8f}" for d in deltas])
        self.writer.writerow(row)
        self.csv_file.flush()

        self.prev_positions = positions.copy()

        if deltas is not None:
            max_abs_delta = np.max(np.abs(deltas))
            self.max_deltas.append(max_abs_delta)

        # Console print every 100 frames
        if self.frame_count % 100 == 0:
            elapsed = time.time() - self.start_time
            recent = self.max_deltas[-100:] if len(self.max_deltas) >= 100 else self.max_deltas
            avg_max = np.mean(recent) if recent else 0
            peak_max = np.max(recent) if recent else 0
            print(f"[{elapsed:6.1f}s] frame={self.frame_count:6d}  "
                  f"avg_max_delta={avg_max:.6f} rad  peak_max_delta={peak_max:.6f} rad")

    def is_done(self) -> bool:
        if self.duration <= 0:
            return False
        return (time.time() - self.start_time) >= self.duration

    def print_summary(self):
        print()
        print("=" * 60)
        print(f"Recording stopped. Total frames: {self.frame_count}")
        if self.max_deltas:
            arr = np.array(self.max_deltas)
            print(f"Per-frame max(|delta|) statistics (rad):")
            print(f"  mean   = {np.mean(arr):.6f}")
            print(f"  median = {np.median(arr):.6f}")
            print(f"  std    = {np.std(arr):.6f}")
            print(f"  p95    = {np.percentile(arr, 95):.6f}")
            print(f"  p99    = {np.percentile(arr, 99):.6f}")
            print(f"  max    = {np.max(arr):.6f}")
            print(f"  min    = {np.min(arr):.6f}")
        print(f"Data saved to: {self.output_path}")

    def close(self):
        try:
            self.node.destroy_subscription(self.sub)
        except Exception:
            pass
        self.csv_file.close()


def main(output: str = "", duration: float = 0):
    if not output:
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        output = f"arm_command_log_{timestamp}.csv"

    node = ros2.ensure_node("record_arm_command")
    recorder = ArmCommandRecorder(node, output, duration)

    period = 0.01
    while ros2.ok() and not recorder.is_done():
        ros2.sleep(period)

    recorder.print_summary()
    recorder.close()


if __name__ == "__main__":
    import tyro
    tyro.cli(main)
