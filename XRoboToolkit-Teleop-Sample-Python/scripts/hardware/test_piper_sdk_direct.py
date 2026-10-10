import math
import time

import tyro

from xrobotoolkit_teleop.hardware.interface.piper_sdk_interface import PiperSDKInterface


def _format_joint_deg(qpos_rad: list[float]) -> list[float]:
    return [round(math.degrees(val), 2) for val in qpos_rad[:6]]


def _print_state(arm: PiperSDKInterface, label: str):
    arm.refresh_state()
    print(f"[{label}]")
    print(f"  joints_deg: {_format_joint_deg(arm.qpos)}")
    print(f"  gripper_m: {round(arm.qpos[6], 4)}")
    print(f"  effort: {[round(val, 3) for val in arm.effort[:6]]}")
    try:
        low_spd = arm.piper.GetArmLowSpdInfoMsgs()
        enable_bits = [
            int(bool(low_spd.motor_1.foc_status.driver_enable_status)),
            int(bool(low_spd.motor_2.foc_status.driver_enable_status)),
            int(bool(low_spd.motor_3.foc_status.driver_enable_status)),
            int(bool(low_spd.motor_4.foc_status.driver_enable_status)),
            int(bool(low_spd.motor_5.foc_status.driver_enable_status)),
            int(bool(low_spd.motor_6.foc_status.driver_enable_status)),
        ]
        print(f"  motor_enable_bits: {enable_bits}")
    except Exception as exc:
        print(f"  motor_enable_bits: unavailable ({exc})")

    try:
        end_pose = arm.piper.GetArmEndPoseMsgs().end_pose
        print(
            "  end_pose_raw:"
            f" x={getattr(end_pose, 'X_axis', 'n/a')}"
            f" y={getattr(end_pose, 'Y_axis', 'n/a')}"
            f" z={getattr(end_pose, 'Z_axis', 'n/a')}"
            f" rx={getattr(end_pose, 'RX_axis', 'n/a')}"
            f" ry={getattr(end_pose, 'RY_axis', 'n/a')}"
            f" rz={getattr(end_pose, 'RZ_axis', 'n/a')}"
        )
    except Exception as exc:
        print(f"  end_pose_raw: unavailable ({exc})")


def _sleep_and_report(arm: PiperSDKInterface, seconds: float, label: str):
    time.sleep(seconds)
    _print_state(arm, label)


def main(
    can_name: str = "can0",
    judge_flag: bool = True,
    joint_speed: int = 30,
    gripper_open_m: float = 0.08,
    gripper_close_m: float = 0.0,
    gripper_effort_n: float = 1.0,
    test_joint_index: int = 2,
    test_joint_delta_deg: float = 8.0,
    step_wait_s: float = 2.0,
    skip_gripper: bool = False,
    skip_go_zero: bool = False,
    skip_joint_test: bool = False,
    end_pose_delta_mm: int = 15,
    skip_end_pose_test: bool = False,
):
    """
    Minimal Piper SDK direct test.

    This script bypasses XR/IK/Placo and only verifies:
    1. SDK connection and enable
    2. Gripper open/close commands
    3. Native go-zero command
    4. A small single-joint motion test
    """
    print("=" * 60)
    print("Piper SDK direct hardware test")
    print("=" * 60)
    print(f"CAN interface: {can_name}")
    print(f"Judge flag: {judge_flag}")
    print(f"Joint speed: {joint_speed}")
    print(f"Test joint index: {test_joint_index}")
    print(f"Test joint delta: {test_joint_delta_deg} deg")
    print(f"Step wait: {step_wait_s} s")
    print(f"End-pose delta: {end_pose_delta_mm} mm")
    print("=" * 60)
    print("Keep the workspace clear before motion starts.")
    print("=" * 60)

    if not 1 <= test_joint_index <= 6:
        raise ValueError("test_joint_index must be in [1, 6].")

    arm = PiperSDKInterface(
        can_name=can_name,
        judge_flag=judge_flag,
        auto_enable=True,
        gripper_exist=True,
        default_joint_speed=joint_speed,
    )

    _print_state(arm, "initial")

    if not skip_gripper:
        print("\n[step] gripper open")
        q_des = list(arm.qpos)
        q_des[6] = max(0.0, min(gripper_open_m, arm.gripper_range_m))
        efforts = [0.0] * 7
        efforts[6] = gripper_effort_n
        arm.set_joint_positions(q_des, efforts=efforts)
        _sleep_and_report(arm, step_wait_s, "after gripper open")

        print("\n[step] gripper close")
        q_des = list(arm.qpos)
        q_des[6] = max(0.0, min(gripper_close_m, arm.gripper_range_m))
        arm.set_joint_positions(q_des, efforts=efforts)
        _sleep_and_report(arm, step_wait_s, "after gripper close")

    if not skip_go_zero:
        print("\n[step] native go_zero")
        arm.go_zero(speed=joint_speed)
        _sleep_and_report(arm, step_wait_s, "after go_zero")

    if not skip_joint_test:
        print("\n[step] single joint move")
        q_start = list(arm.qpos)
        q_target = list(q_start)
        q_target[test_joint_index - 1] += math.radians(test_joint_delta_deg)
        arm.set_joint_positions(q_target)
        _sleep_and_report(arm, step_wait_s, "after joint move")

        print("\n[step] single joint move back")
        arm.set_joint_positions(q_start)
        _sleep_and_report(arm, step_wait_s, "after joint move back")

    if not skip_end_pose_test:
        print("\n[step] end-pose move in +X")
        try:
            try:
                arm.piper.MotionCtrl_2(0x01, 0x00, joint_speed, 0)
            except TypeError:
                arm.piper.MotionCtrl_2(0x01, 0x00, joint_speed)
            end_pose = arm.piper.GetArmEndPoseMsgs().end_pose
            x = int(getattr(end_pose, "X_axis")) + int(end_pose_delta_mm * 1000)
            y = int(getattr(end_pose, "Y_axis"))
            z = int(getattr(end_pose, "Z_axis"))
            rx = int(getattr(end_pose, "RX_axis"))
            ry = int(getattr(end_pose, "RY_axis"))
            rz = int(getattr(end_pose, "RZ_axis"))
            print(f"  sending EndPoseCtrl to x={x}, y={y}, z={z}, rx={rx}, ry={ry}, rz={rz}")
            arm.piper.EndPoseCtrl(x, y, z, rx, ry, rz)
        except Exception as exc:
            print(f"  EndPoseCtrl send failed: {exc}")
        _sleep_and_report(arm, step_wait_s, "after end-pose move")

    print("\nTest sequence finished.")
    arm.stop()


if __name__ == "__main__":
    tyro.cli(main)
