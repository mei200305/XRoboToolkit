#!/usr/bin/env python3
"""
Analyze recorded /throw/arm_joint_command CSV to identify jitter sources.

Usage:
  python scripts/utils/analyze_jitter.py arm_command_log_20260721_141226.csv
"""

import sys

import numpy as np

JOINT_NAMES = [
    "arml_1_joint", "arml_2_joint", "arml_3_joint", "arml_4_joint",
    "arml_5_joint", "arml_6_joint", "arml_7_joint",
    "armr_1_joint", "armr_2_joint", "armr_3_joint", "armr_4_joint",
    "armr_5_joint", "armr_6_joint", "armr_7_joint",
]
N_JOINTS = len(JOINT_NAMES)


def load_csv(path: str):
    """Load CSV, return positions (N, 14) and deltas (N, 14)."""
    data = np.genfromtxt(path, delimiter=",", skip_header=1, dtype=str)
    # Columns: secs(0), nsecs(1), 14 pos(2:16), 14 delta(16:30)
    positions = data[:, 2:2 + N_JOINTS].astype(float)
    deltas = data[:, 16:16 + N_JOINTS].astype(float)
    return positions, deltas


def analyze(path: str):
    positions, deltas = load_csv(path)
    n_frames = len(deltas)

    print(f"Total frames: {n_frames}")
    print()

    # 1. Per-joint delta statistics
    print("=" * 80)
    print("Per-joint |delta| statistics (rad):")
    print(f"{'Joint':<30s} {'mean':>10s} {'median':>10s} {'std':>10s} {'p95':>10s} {'p99':>10s} {'max':>10s}")
    print("-" * 80)
    for i, name in enumerate(JOINT_NAMES):
        abs_d = np.abs(deltas[:, i])
        print(f"{name:<30s} {np.mean(abs_d):10.6f} {np.median(abs_d):10.6f} "
              f"{np.std(abs_d):10.6f} {np.percentile(abs_d,95):10.6f} "
              f"{np.percentile(abs_d,99):10.6f} {np.max(abs_d):10.6f}")
    print()

    # 2. Segment analysis: static vs moving
    # "Static" = max(|delta|) across all joints < threshold
    frame_max_delta = np.max(np.abs(deltas), axis=1)
    static_thresholds = [0.0005, 0.001, 0.002, 0.005]
    print("=" * 80)
    print("Static-frame analysis (controller held still):")
    print(f"{'Threshold (rad)':>16s} {'frames':>8s} {'%':>8s}  {'mean max|d|':>14s}")
    print("-" * 60)
    for thresh in static_thresholds:
        static_mask = frame_max_delta < thresh
        n_static = np.sum(static_mask)
        if n_static > 0:
            mean_static = np.mean(frame_max_delta[static_mask])
        else:
            mean_static = 0
        print(f"{thresh:16.6f} {n_static:8d} {100*n_static/n_frames:7.1f}%  {mean_static:14.8f}")
    print()

    # 3. Run-length of consecutive static frames
    # How long does the arm "hold still" before a jitter spike?
    static_mask = frame_max_delta < 0.001
    run_lengths = []
    current_run = 0
    for m in static_mask:
        if m:
            current_run += 1
        else:
            if current_run > 0:
                run_lengths.append(current_run)
            current_run = 0
    if current_run > 0:
        run_lengths.append(current_run)

    if run_lengths:
        rl = np.array(run_lengths)
        print("=" * 80)
        print("Consecutive static-frame run lengths (threshold=0.001 rad):")
        print(f"  num_runs={len(rl)}  mean={np.mean(rl):.1f} frames  median={np.median(rl):.1f}  "
              f"max={np.max(rl)}  std={np.std(rl):.1f}")
        print(f"  (At ~100Hz, 10 frames = 0.1s, 100 frames = 1s)")
    print()

    # 4. Frequency analysis: FFT of the first joint for static periods
    # Find a long static segment
    static_mask = frame_max_delta < 0.002
    runs = []
    run_start = None
    for i, m in enumerate(static_mask):
        if m and run_start is None:
            run_start = i
        elif not m and run_start is not None:
            runs.append((run_start, i))
            run_start = None
    if run_start is not None:
        runs.append((run_start, len(static_mask)))

    long_runs = [(s, e) for s, e in runs if (e - s) >= 100]
    if long_runs:
        print("=" * 80)
        print("FFT analysis of longest static segment (position signal):")
        best = max(long_runs, key=lambda x: x[1] - x[0])
        segment = positions[best[0]:best[1], 0]  # first joint
        segment = segment - np.mean(segment)  # remove DC

        fft = np.abs(np.fft.rfft(segment))
        freqs = np.fft.rfftfreq(len(segment), d=0.01)  # assuming 100Hz
        # Show top frequency components
        top_idx = np.argsort(fft)[-10:][::-1]
        print(f"  Segment: frames {best[0]}-{best[1]} ({len(segment)} frames ≈ {len(segment)*0.01:.1f}s)")
        print(f"  Top frequency components (Hz):")
        for idx in top_idx[:8]:
            if fft[idx] > 1e-6:
                print(f"    {freqs[idx]:6.2f} Hz  magnitude={fft[idx]:.4f}")
    else:
        print("No long static segment found for FFT analysis.")

    # 5. Delta distribution histogram summary
    all_abs_deltas = np.abs(deltas).flatten()
    all_abs_deltas = all_abs_deltas[all_abs_deltas > 1e-9]  # skip exact zeros
    print()
    print("=" * 80)
    print("Delta magnitude distribution (non-zero deltas, all joints):")
    percentiles = [50, 75, 90, 95, 99, 99.9]
    for p in percentiles:
        print(f"  p{p:5.1f}: {np.percentile(all_abs_deltas, p):.8f} rad")


if __name__ == "__main__":
    if len(sys.argv) < 2:
        print(f"Usage: python {sys.argv[0]} <csv_file>")
        sys.exit(1)
    analyze(sys.argv[1])
