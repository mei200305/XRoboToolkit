import importlib
import math
import time
from typing import Iterable, Optional


class PiperSDKInterface:
    """
    Direct Piper hardware interface based on the local piper_sdk package.

    This wrapper intentionally follows the data conversions used by
    `piper_ros/src/piper/scripts/piper_ctrl_single_node.py`, because that is the
    most concrete reference available in this environment.
    """

    def __init__(
        self,
        can_name: str = "can0",
        judge_flag: bool = True,
        auto_enable: bool = True,
        gripper_exist: bool = True,
        gripper_range_m: float = 0.08,
        default_joint_speed: int = 50,
        prefer_legacy_interface: bool = True,
    ):
        self.can_name = can_name
        self.judge_flag = judge_flag
        self.auto_enable = auto_enable
        self.gripper_exist = gripper_exist
        self.gripper_range_m = gripper_range_m
        self.default_joint_speed = default_joint_speed
        self.prefer_legacy_interface = prefer_legacy_interface
        self._last_published_speed = None
        self._last_debug_print_ts = 0.0

        self.piper = self._create_sdk_instance()
        self._connect()

        self.qpos = [0.0] * 7
        self.qvel = [0.0] * 7
        self.effort = [0.0] * 7
        self.timestamp = 0.0
        self.q_des = None

        if self.auto_enable:
            self.enable()

    def _create_sdk_instance(self):
        last_error = None

        if self.prefer_legacy_interface:
            try:
                module = importlib.import_module("piper_sdk")
                cls = getattr(module, "C_PiperInterface")
                print("Using legacy piper_sdk.C_PiperInterface backend.")
                return cls(can_name=self.can_name)
            except Exception as exc:
                last_error = exc

            try:
                module = importlib.import_module("piper_sdk.interface.piper_interface_v2")
                cls = getattr(module, "C_PiperInterface_V2")
                print("Using piper_sdk C_PiperInterface_V2 backend.")
                return cls.get_instance(can_name=self.can_name, judge_flag=self.judge_flag)
            except Exception as exc:
                last_error = exc
        else:
            try:
                module = importlib.import_module("piper_sdk.interface.piper_interface_v2")
                cls = getattr(module, "C_PiperInterface_V2")
                print("Using piper_sdk C_PiperInterface_V2 backend.")
                return cls.get_instance(can_name=self.can_name, judge_flag=self.judge_flag)
            except Exception as exc:
                last_error = exc

            try:
                module = importlib.import_module("piper_sdk")
                cls = getattr(module, "C_PiperInterface")
                print("Using legacy piper_sdk.C_PiperInterface backend.")
                return cls(can_name=self.can_name)
            except Exception as exc:
                last_error = exc

        raise ImportError(
            "Failed to import piper_sdk. Install the SDK in the active environment first."
        ) from last_error

    def _connect(self):
        if hasattr(self.piper, "ConnectPort"):
            self.piper.ConnectPort()
        if hasattr(self.piper, "connect"):
            self.piper.connect()

        # Keep the same startup motion mode used by piper_ros.
        if hasattr(self.piper, "MotionCtrl_2"):
            try:
                self.piper.MotionCtrl_2(0x01, 0x01, 30, 0)
            except TypeError:
                self.piper.MotionCtrl_2(0x01, 0x01, 30)

    def enable(self):
        timeout_s = 5.0
        start_time = time.time()

        while True:
            enable_list = self._get_enable_status_list()
            enabled = bool(enable_list) and all(enable_list)
            print(f"Piper enable status: {enable_list} -> all_enabled={enabled}")
            if enabled:
                return True

            if hasattr(self.piper, "EnableArm"):
                try:
                    self.piper.EnableArm(7)
                except TypeError:
                    self.piper.EnableArm()
            if self.gripper_exist and hasattr(self.piper, "GripperCtrl"):
                self.piper.GripperCtrl(0, 1000, 0x01, 0)

            if time.time() - start_time > timeout_s:
                print("WARNING: Piper enable timed out after 5 seconds.")
                return False

            time.sleep(1.0)

    def disable(self):
        if hasattr(self.piper, "DisableArm"):
            try:
                self.piper.DisableArm(7)
            except TypeError:
                self.piper.DisableArm()
        if self.gripper_exist and hasattr(self.piper, "GripperCtrl"):
            self.piper.GripperCtrl(0, 1000, 0x00, 0)

    def refresh_state(self):
        joint_msgs = self.piper.GetArmJointMsgs()
        gripper_msgs = self.piper.GetArmGripperMsgs()
        high_spd_msgs = self.piper.GetArmHighSpdInfoMsgs()

        deg_to_rad = math.pi / 180.0 / 1000.0
        self.qpos = [
            joint_msgs.joint_state.joint_1 * deg_to_rad,
            joint_msgs.joint_state.joint_2 * deg_to_rad,
            joint_msgs.joint_state.joint_3 * deg_to_rad,
            joint_msgs.joint_state.joint_4 * deg_to_rad,
            joint_msgs.joint_state.joint_5 * deg_to_rad,
            joint_msgs.joint_state.joint_6 * deg_to_rad,
            gripper_msgs.gripper_state.grippers_angle / 1000000.0,
        ]
        self.qvel = [
            high_spd_msgs.motor_1.motor_speed / 1000.0,
            high_spd_msgs.motor_2.motor_speed / 1000.0,
            high_spd_msgs.motor_3.motor_speed / 1000.0,
            high_spd_msgs.motor_4.motor_speed / 1000.0,
            high_spd_msgs.motor_5.motor_speed / 1000.0,
            high_spd_msgs.motor_6.motor_speed / 1000.0,
            0.0,
        ]
        self.effort = [
            high_spd_msgs.motor_1.effort / 1000.0,
            high_spd_msgs.motor_2.effort / 1000.0,
            high_spd_msgs.motor_3.effort / 1000.0,
            high_spd_msgs.motor_4.effort / 1000.0,
            high_spd_msgs.motor_5.effort / 1000.0,
            high_spd_msgs.motor_6.effort / 1000.0,
            gripper_msgs.gripper_state.grippers_effort / 1000.0,
        ]
        self.timestamp = time.time()

        if self.q_des is None:
            self.q_des = list(self.qpos)

        return self.qpos

    def _get_enable_status_list(self):
        try:
            low_spd = self.piper.GetArmLowSpdInfoMsgs()
            return [
                bool(low_spd.motor_1.foc_status.driver_enable_status),
                bool(low_spd.motor_2.foc_status.driver_enable_status),
                bool(low_spd.motor_3.foc_status.driver_enable_status),
                bool(low_spd.motor_4.foc_status.driver_enable_status),
                bool(low_spd.motor_5.foc_status.driver_enable_status),
                bool(low_spd.motor_6.foc_status.driver_enable_status),
            ]
        except Exception:
            return []

    def set_joint_positions(
        self,
        positions: Iterable[float],
        velocities: Optional[Iterable[float]] = None,
        efforts: Optional[Iterable[float]] = None,
    ):
        q_des = list(positions)
        if len(q_des) < 7:
            q_des.extend([0.0] * (7 - len(q_des)))
        self.q_des = q_des[:7]

        speed = self.default_joint_speed
        if velocities is not None:
            velocities = list(velocities)
            if len(velocities) >= 7:
                speed = int(max(0, min(100, round(velocities[6]))))
        self._publish_speed(speed)

        factor = 1000.0 * 180.0 / math.pi
        joint_values = [int(round(val * factor)) for val in self.q_des[:6]]
        self.piper.JointCtrl(*joint_values)

        if self.gripper_exist and hasattr(self.piper, "GripperCtrl"):
            gripper_pos = int(round(max(0.0, min(self.q_des[6], self.gripper_range_m)) * 1000000.0))
            gripper_effort = 1000
            if efforts is not None:
                efforts = list(efforts)
                if len(efforts) >= 7:
                    gripper_effort = int(round(max(0.5, min(efforts[6], 3.0)) * 1000.0))
            self.piper.GripperCtrl(abs(gripper_pos), gripper_effort, 0x01, 0)

        now = time.time()
        if now - self._last_debug_print_ts > 1.0:
            self._last_debug_print_ts = now
            print(
                "Piper SDK cmd:",
                f"joint_deg_x1000={joint_values}",
                f"gripper_um={int(round(max(0.0, min(self.q_des[6], self.gripper_range_m)) * 1000000.0))}",
                f"speed={speed}",
            )

    def go_zero(self, speed: Optional[int] = None, is_mit_mode: bool = False):
        speed = self.default_joint_speed if speed is None else int(max(0, min(100, speed)))
        mit_mode = 0xAD if is_mit_mode else 0
        if hasattr(self.piper, "MotionCtrl_2"):
            try:
                self.piper.MotionCtrl_2(0x01, 0x01, speed, mit_mode)
            except TypeError:
                self.piper.MotionCtrl_2(0x01, 0x01, speed)
            self._last_published_speed = speed
        self.piper.JointCtrl(0, 0, 0, 0, 0, 0)
        print(f"Piper SDK go_zero sent with speed={speed}, mit_mode={mit_mode}.")

    def _publish_speed(self, speed: int):
        if speed == self._last_published_speed:
            return
        if hasattr(self.piper, "MotionCtrl_2"):
            try:
                self.piper.MotionCtrl_2(0x01, 0x01, speed, 0)
            except TypeError:
                self.piper.MotionCtrl_2(0x01, 0x01, speed)
            self._last_published_speed = speed

    def stop(self):
        # Do not send MotionCtrl_1 stop here. It leaves Piper in a stopped
        # state, so the next process may connect and enable successfully but
        # ignore JointCtrl commands until a reset/recover command is sent.
        pass
