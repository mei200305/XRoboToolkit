#!/usr/bin/env python3
"""
关节空间五次投壶 + 段2 末端速度最大时刻松开夹爪。

基于 throw_joint_quintic_trajectory.py，执行时在段2 |v|_max 发布:
  话题 hand_control (std_msgs/String)  内容 "open"
与 hand/hand_command_node.py 配套使用（需另开终端运行 hand_command_node）。

用法:
  # 终端1: roscore
  # 终端2: cd hand && python3 hand_command_node.py
  # 终端3: throw_RL (MANUAL)
  python3 throw_joint_quintic_hand_release.py --dry-run
  python3 throw_joint_quintic_hand_release.py --joint-topic-only

  # 投掷前可先合爪（可选）
  python3 throw_joint_quintic_hand_release.py --joint-topic-only --close-hand-at-start
"""

from __future__ import annotations

import argparse
import inspect

import numpy as np

from arm_trajectory_pipeline import plot_pose_trajectory_all
from throw_ee_trajectory import RightArmFK

import throw_joint_quintic_trajectory as _tjt

END_JOINT = _tjt.END_JOINT
PREP_JOINT = _tjt.PREP_JOINT
StitchedJointQuinticTrajectory = _tjt.StitchedJointQuinticTrajectory
build_joint_quintic_trajectory = _tjt.build_joint_quintic_trajectory

_BUILD_SUPPORTS_CUSTOM_JOINTS = (
    "prep_joint" in inspect.signature(StitchedJointQuinticTrajectory.build).parameters
)


def build_stitched_trajectory(
    *,
    T_zero_via: float,
    T_via_prep: float,
    T_prep_end: float,
    T_end_zero: float,
    prep_joint: np.ndarray | None = None,
    end_joint: np.ndarray | None = None,
    n_samples: int = 500,
) -> StitchedJointQuinticTrajectory:
    """兼容旧版 build()（无 prep_joint/end_joint 参数）。"""
    prep = np.asarray(prep_joint if prep_joint is not None else PREP_JOINT, dtype=float)
    end = np.asarray(end_joint if end_joint is not None else END_JOINT, dtype=float)

    if _BUILD_SUPPORTS_CUSTOM_JOINTS:
        return StitchedJointQuinticTrajectory.build(
            T_zero_via=T_zero_via,
            T_via_prep=T_via_prep,
            T_prep_end=T_prep_end,
            T_end_zero=T_end_zero,
            n_samples=n_samples,
            prep_joint=prep,
            end_joint=end,
        )

    old_prep = np.asarray(_tjt.PREP_JOINT, dtype=float).copy()
    old_end = np.asarray(_tjt.END_JOINT, dtype=float).copy()
    try:
        _tjt.PREP_JOINT = prep
        _tjt.END_JOINT = end
        return StitchedJointQuinticTrajectory.build(
            T_zero_via=T_zero_via,
            T_via_prep=T_via_prep,
            T_prep_end=T_prep_end,
            T_end_zero=T_end_zero,
            n_samples=n_samples,
        )
    finally:
        _tjt.PREP_JOINT = old_prep
        _tjt.END_JOINT = old_end


if not _BUILD_SUPPORTS_CUSTOM_JOINTS:
    print(
        "[提示] 实机 throw_joint_quintic_trajectory.py 为旧版（build 无 prep_joint），"
        "Δq1 补偿通过临时覆盖关键帧实现"
    )

# 与 hand/hand_command_node.py 一致
DEFAULT_HAND_TOPIC = "hand_control"
HAND_CMD_OPEN = "open"
HAND_CMD_CLOSE = "close"


def find_segment2_vmax_index(traj, fk: RightArmFK | None = None) -> tuple[int, float]:
    """段2 末端笛卡尔速度最大时刻。返回 (索引, |v| m/s)。"""
    from arm_trajectory_pipeline import cartesian_pva_from_joint_trajectory

    fk = fk or RightArmFK.load()
    m2 = np.asarray(traj.phase) == 2
    if not np.any(m2):
        return 0, 0.0
    _, vel, _ = cartesian_pva_from_joint_trajectory(traj, fk=fk)
    speeds = np.linalg.norm(vel[m2], axis=1)
    idx_local = int(np.argmax(speeds))
    global_idx = int(np.where(m2)[0][idx_local])
    return global_idx, float(speeds[idx_local])


def compute_release_index(
    traj,
    vmax_idx: int,
    *,
    advance_s: float = 0.0,
    delay_s: float = 0.0,
) -> int:
    """
    松手时刻: t_release = t_v_max - advance_s + delay_s

    - advance_s > 0: 提前松手（补偿夹爪慢；但出手速度更低 → 距离更近）
    - delay_s > 0:   延后松手（出手速度更高 → 距离更远；夹爪需足够快）
    限制在段2 范围内。
    """
    m2 = np.asarray(traj.phase) == 2
    if not np.any(m2):
        return vmax_idx

    phase2_indices = np.where(m2)[0]
    phase2_start = int(phase2_indices[0])
    phase2_end = int(phase2_indices[-1])
    t_release = float(traj.t[vmax_idx]) - advance_s + delay_s
    release_idx = int(np.searchsorted(traj.t, t_release, side="left"))
    return max(phase2_start, min(release_idx, phase2_end))


def run_throw_with_hand_release(
    traj,
    release_index: int,
    *,
    joint_topic_only: bool = True,
    joint_cmd_topic: str = "/throw/arm_joint_command",
    hand_topic: str = DEFAULT_HAND_TOPIC,
    hand_release: bool = True,
    close_hand_at_start: bool = False,
    hz: float = 500.0,
    step_index: int = 1,
    zero_publish_vel: bool = False,
    **pt_kwargs,
) -> None:
    """执行轨迹；到达 release_index 时发布 hand_control/open。"""
    from sensor_msgs.msg import JointState
    from std_msgs.msg import String

    from xrobotoolkit_teleop.hardware.interface import ros2_compat as ros2

    try:
        from arm_joint_topics import build_arm_joint_command_msg
    except ImportError:
        def build_arm_joint_command_msg(q_left, qd_left, q_right, qd_right, *, stamp=None, **kwargs):
            from sensor_msgs.msg import JointState

            msg = JointState()
            if stamp is not None:
                msg.header.stamp = stamp
            msg.name = []
            msg.position = list(q_left) + list(q_right) + [0.0] * 14
            msg.velocity = list(qd_left) + list(qd_right) + [0.0] * 14
            return msg

    node = ros2.ensure_node("throw_quintic_hand_release")

    hand_pub = (
        node.create_publisher(String, hand_topic, 1)
        if hand_release or close_hand_at_start
        else None
    )
    if close_hand_at_start and hand_pub is not None:
        ros2.sleep(0.2)
        close_msg = String()
        close_msg.data = HAND_CMD_CLOSE
        hand_pub.publish(close_msg)
        print(f"[夹爪] 起始合爪 -> {hand_topic}/{HAND_CMD_CLOSE}")

    if joint_topic_only:
        pub = node.create_publisher(JointState, joint_cmd_topic, 10)
        period = 1.0 / hz if hz > 0 else 0.002
        n = len(traj.t)
        idx = 0
        released = False
        print(
            f"[关节话题] {joint_cmd_topic} @ {hz}Hz  "
            f"松手 idx={release_index} t≈{traj.t[release_index]:.3f}s -> {hand_topic}/{HAND_CMD_OPEN}"
        )
        while ros2.ok() and idx < n:
            if hand_release and hand_pub is not None and not released and idx >= release_index:
                open_msg = String()
                open_msg.data = HAND_CMD_OPEN
                hand_pub.publish(open_msg)
                released = True
                print(
                    f"  [夹爪] 松手 t={traj.t[idx]:.3f}s 发布 {HAND_CMD_OPEN} -> {hand_topic}"
                )

            q_right = np.asarray(traj.joint_q[idx], dtype=float).reshape(7)
            qd_right = (
                np.zeros(7, dtype=float)
                if zero_publish_vel
                else np.asarray(traj.joint_qd[idx], dtype=float).reshape(7)
            )
            msg = build_arm_joint_command_msg(
                [0.0] * 7,
                [0.0] * 7,
                q_right.tolist(),
                qd_right.tolist(),
                stamp=ros2.now_msg(node),
            )
            pub.publish(msg)
            if idx % int(hz * 2) == 0:
                print(f"  t={traj.t[idx]:.2f}s phase={traj.phase[idx]} q0={q_right[0]:+.3f}")
            idx += step_index
            ros2.sleep(period)
        print("关节话题发布完成")
        return

    from execute_throw_trajectory import (
        ROS_HZ,
        ReceivedData,
        _import_ros_deps,
        build_arm_joint_state_msg,
        build_pt_command,
        wait_joint_state,
    )
    from arm_torque_control import ControlMode, compute_trajectory_torque_ff

    _, Float64MultiArray, JointState, Q2WMsgPublishers = _import_ros_deps()

    mode = ControlMode(pt_kwargs.get("control_mode", "position"))
    kp_arm = pt_kwargs.get("kp_arm", 80.0)
    kd_arm = pt_kwargs.get("kd_arm", 1.0)
    publish_joint_cmd = pt_kwargs.get("publish_joint_cmd", False)
    zero_left = pt_kwargs.get("zero_left_arm", False)

    Q2WMsgPublishers.init()
    joint_cmd_pub = None
    if publish_joint_cmd:
        joint_cmd_pub = node.create_publisher(JointState, joint_cmd_topic, 10)

    period = 1.0 / ROS_HZ if ROS_HZ > 0 else 0.01
    box = ReceivedData()
    print("等待关节状态...")
    pos_cur = wait_joint_state(box)

    torque_ff = None
    if mode in (ControlMode.IMPEDANCE, ControlMode.TORQUE):
        torque_ff = compute_trajectory_torque_ff(
            traj.joint_q, traj.joint_qd, traj.joint_qdd
        )

    n = len(traj.t)
    idx = 0
    released = False
    print(f"PT 下发 + 松手 idx={release_index} t≈{traj.t[release_index]:.3f}s")

    while ros2.ok() and idx < n:
        if hand_release and hand_pub is not None and not released and idx >= release_index:
            open_msg = String()
            open_msg.data = HAND_CMD_OPEN
            hand_pub.publish(open_msg)
            released = True
            print(f"  [夹爪] 松手 t={traj.t[idx]:.3f}s 发布 {HAND_CMD_OPEN} -> {hand_topic}")

        stamp = ros2.now_msg(node)
        msg = Float64MultiArray()
        msg.data = build_pt_command(
            idx,
            traj,
            pos_cur,
            mode=mode,
            kp_arm=kp_arm,
            kd_arm=kd_arm,
            torque_ff=torque_ff,
            zero_left_arm=zero_left,
            zero_publish_vel=zero_publish_vel,
        )
        Q2WMsgPublishers.upper_motor_pt_control_data.publish(msg)

        if joint_cmd_pub is not None:
            joint_cmd_pub.publish(
                build_arm_joint_state_msg(
                    idx, traj, stamp=stamp, zero_left_arm=True, zero_publish_vel=zero_publish_vel
                )
            )

        idx += step_index
        ros2.sleep(period)

    print("轨迹执行完成")


def main() -> None:
    parser = argparse.ArgumentParser(
        description="关节五次投壶 + 段2 v_max 时夹爪 open (hand_command_node)"
    )
    parser.add_argument("--hz", type=float, default=500.0)
    parser.add_argument("--T-zero-via", type=float, default=1.0)
    parser.add_argument("--T-via-prep", type=float, default=7.5)
    parser.add_argument("--T-prep-end", type=float, default=0.63)
    parser.add_argument("--T-end-zero", type=float, default=3.5)
    parser.add_argument("--save", type=str, default="throw_joint_quintic_hand_traj.npz")
    parser.add_argument("--joint-plot", type=str, default="throw_joint_quintic_hand_joint.png")
    parser.add_argument("--pva-plot", type=str, default="throw_joint_quintic_hand_pva.png")
    parser.add_argument("--dry-run", action="store_true")
    parser.add_argument("--step", type=int, default=1)
    parser.add_argument("--kp", type=float, default=80.0)
    parser.add_argument("--kd", type=float, default=1.0)
    parser.add_argument("--control-mode", choices=["position", "impedance", "torque"], default="position")
    parser.add_argument("--joint-cmd-topic", type=str, default="/throw/arm_joint_command")
    parser.add_argument("--joint-topic-only", action="store_true")
    parser.add_argument("--no-joint-topic", action="store_true")
    parser.add_argument("--zero-vel", action="store_true")
    parser.add_argument("--delta-q1", type=float, default=0.0)
    parser.add_argument(
        "--hand-topic",
        type=str,
        default=DEFAULT_HAND_TOPIC,
        help="夹爪话题，与 hand_command_node 一致",
    )
    parser.add_argument(
        "--no-hand-release",
        action="store_true",
        help="不发布 open（仅当调试轨迹时用）",
    )
    parser.add_argument(
        "--close-hand-at-start",
        action="store_true",
        help="执行开始前先发 close 合爪",
    )
    parser.add_argument(
        "--release-index",
        type=int,
        default=None,
        help="手动指定松手轨迹索引（默认 v_max 再提前 --release-advance）",
    )
    parser.add_argument(
        "--release-advance",
        type=float,
        default=0.0,
        help="相对 v_max 提前松手 (s)；提前→出手慢、距离近，仅夹爪很慢时用",
    )
    parser.add_argument(
        "--release-delay",
        type=float,
        default=0.0,
        help="相对 v_max 延后松手 (s)；延后→出手快、距离远，默认 0（与 v_max 对齐）",
    )
    args = parser.parse_args()

    fk = RightArmFK.load()
    prep_joint = PREP_JOINT.copy()
    end_joint = END_JOINT.copy()
    prep_joint[1] += args.delta_q1
    end_joint[1] += args.delta_q1
    if abs(args.delta_q1) > 1e-9:
        print(f"j1 侧向补偿: Δq1={args.delta_q1:+.4f} rad")

    stitched = build_stitched_trajectory(
        T_zero_via=args.T_zero_via,
        T_via_prep=args.T_via_prep,
        T_prep_end=args.T_prep_end,
        T_end_zero=args.T_end_zero,
        prep_joint=prep_joint,
        end_joint=end_joint,
    )
    traj = build_joint_quintic_trajectory(stitched=stitched, hz=args.hz, fk=fk)

    vmax_idx, vmax_speed = find_segment2_vmax_index(traj, fk=fk)
    if args.release_index is not None:
        release_idx = args.release_index
    else:
        release_idx = compute_release_index(
            traj,
            vmax_idx,
            advance_s=args.release_advance,
            delay_s=args.release_delay,
        )

    T1, T2, T3 = stitched.segment_T
    net_ms = (args.release_delay - args.release_advance) * 1000.0
    print(
        f"轨迹总时长: {T1 + T2 + T3:.2f}s  "
        f"段2 T_prep_end={T2:.2f}s  "
        f"v_max: idx={vmax_idx} t={traj.t[vmax_idx]:.3f}s |v|={vmax_speed:.3f} m/s"
    )
    if release_idx == vmax_idx and abs(net_ms) < 1e-6:
        print(
            f"  松手(=v_max): idx={release_idx} t={traj.t[release_idx]:.3f}s  "
            f"phase={traj.phase[release_idx]}"
        )
    else:
        print(
            f"  松手: idx={release_idx} t={traj.t[release_idx]:.3f}s  "
            f"相对v_max {net_ms:+.0f}ms  phase={traj.phase[release_idx]}"
        )
    stitched.report(fk)

    np.savez(
        args.save,
        t=traj.t,
        pos=traj.pos,
        phase=traj.phase,
        joint_q=traj.joint_q,
        joint_qd=traj.joint_qd,
        joint_qdd=traj.joint_qdd,
        prep_joint=prep_joint,
        end_joint=end_joint,
        T_prep_end=args.T_prep_end,
        release_index=release_idx,
        release_time=traj.t[release_idx],
        release_advance_s=args.release_advance,
        release_delay_s=args.release_delay,
        vmax_index=vmax_idx,
        vmax_time=traj.t[vmax_idx],
        vmax_speed=vmax_speed,
        delta_q1=args.delta_q1,
    )
    print(f"已保存: {args.save}  N={len(traj.t)}")

    if args.pva_plot or args.joint_plot:
        plot_pose_trajectory_all(
            traj,
            cartesian_path=args.pva_plot or None,
            joint_path=args.joint_plot or None,
        )

    if args.dry_run:
        return

    hand_release = not args.no_hand_release
    pt_kw = {
        "kp_arm": args.kp,
        "control_mode": args.control_mode,
        "publish_joint_cmd": not args.no_joint_topic,
    }
    sig = inspect.signature(
        __import__("execute_throw_trajectory", fromlist=["run_ros_execution"]).run_ros_execution
    )
    if "kd_arm" in sig.parameters:
        pt_kw["kd_arm"] = args.kd

    run_throw_with_hand_release(
        traj,
        release_idx,
        joint_topic_only=args.joint_topic_only,
        joint_cmd_topic=args.joint_cmd_topic,
        hand_topic=args.hand_topic,
        hand_release=hand_release,
        close_hand_at_start=args.close_hand_at_start,
        hz=args.hz,
        step_index=args.step,
        zero_publish_vel=args.zero_vel,
        **pt_kw,
    )


if __name__ == "__main__":
    main()
