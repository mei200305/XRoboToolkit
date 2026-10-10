import tyro

from xrobotoolkit_teleop.hardware.piper_teleop_controller import (
    DEFAULT_MANIPULATOR_CONFIG,
    DEFAULT_PIPER_ARM_JOINT_NAMES,
    DEFAULT_PIPER_JOINT_NAMES,
    DEFAULT_PIPER_URDF_PATH,
    PiperTeleopController,
)
from xrobotoolkit_teleop.hardware.piper_sdk_teleop_controller import PiperSDKTeleopController


def main(
    backend: str = "sdk",
    robot_urdf_path: str = DEFAULT_PIPER_URDF_PATH,
    link_name: str = "link6",
    scale_factor: float = 1.0,
    enable_log_data: bool = True,
    enable_camera: bool = False,
    visualize_placo: bool = False,
    control_rate_hz: int = 100,
    log_dir: str = "logs/piper",
    joint_names: list[str] = DEFAULT_PIPER_JOINT_NAMES,
    arm_joint_names: list[str] = DEFAULT_PIPER_ARM_JOINT_NAMES,
    joint_control_topic: str = "/joint_states",
    joint_state_topic: str = "/joint_states_single",
    enable_topic: str = "/enable_flag",
    namespace: str = "",
    can_name: str = "can0",
    judge_flag: bool = True,
    auto_enable: bool = True,
    piper_joint_speed: int = 50,
    command_filter_alpha: float = 0.35,
    max_joint_velocity_rad_s: float = 1.5,
    max_gripper_velocity_m_s: float = 0.08,
    disable_on_shutdown: bool = False,
):
    """
    Run single-arm Piper hardware teleoperation.

    `backend="sdk"` uses the local `piper_sdk` CAN interface directly.
    `backend="ros"` uses ROS topics published by `piper_ros`.
    """
    manipulator_config = {
        "right_arm": {
            **DEFAULT_MANIPULATOR_CONFIG["right_arm"],
            "link_name": link_name,
        }
    }

    backend = backend.lower().strip()
    if backend not in {"sdk", "ros"}:
        raise ValueError(f"Unsupported backend: {backend}. Expected 'sdk' or 'ros'.")

    print("=" * 60)
    print("Piper single-arm hardware teleoperation")
    print("=" * 60)
    print(f"Backend: {backend}")
    print(f"URDF: {robot_urdf_path}")
    print(f"End effector link: {link_name}")
    print(f"Control rate: {control_rate_hz} Hz")
    print(f"Record camera topics: {enable_camera}")
    if backend == "sdk":
        print(f"CAN interface: {can_name}")
        print(f"Judge flag: {judge_flag}")
        print(f"Piper joint speed: {piper_joint_speed}")
        print(f"Command filter alpha: {command_filter_alpha}")
        print(f"Max joint velocity: {max_joint_velocity_rad_s} rad/s")
    else:
        print(f"Joint control topic: {joint_control_topic}")
        print(f"Joint state topic: {joint_state_topic}")
        print(f"Enable topic: {enable_topic}")
        if namespace:
            print(f"Namespace: {namespace}")
    print("右手控制器 -> Piper 单臂真机")
    print("按住 right grip 激活控制")
    print("按 A 键让 Piper 回到零点")
    print("right trigger 控制夹爪开合")
    print("按 B 键可开始/结束日志录制")
    print(f"退出时失能机械臂: {disable_on_shutdown}")
    print("按 Ctrl+C 退出")
    print("=" * 60)

    if backend == "sdk":
        controller = PiperSDKTeleopController(
            robot_urdf_path=robot_urdf_path,
            manipulator_config=manipulator_config,
            joint_names=joint_names,
            arm_joint_names=arm_joint_names,
            can_name=can_name,
            judge_flag=judge_flag,
            scale_factor=scale_factor,
            enable_log_data=enable_log_data,
            visualize_placo=visualize_placo,
            control_rate_hz=control_rate_hz,
            log_dir=log_dir,
            enable_camera=enable_camera,
            auto_enable=auto_enable,
            piper_joint_speed=piper_joint_speed,
            command_filter_alpha=command_filter_alpha,
            max_joint_velocity_rad_s=max_joint_velocity_rad_s,
            max_gripper_velocity_m_s=max_gripper_velocity_m_s,
            disable_on_shutdown=disable_on_shutdown,
        )
    else:
        controller = PiperTeleopController(
            robot_urdf_path=robot_urdf_path,
            manipulator_config=manipulator_config,
            joint_names=joint_names,
            arm_joint_names=arm_joint_names,
            joint_control_topic=joint_control_topic,
            joint_state_topic=joint_state_topic,
            enable_topic=enable_topic,
            namespace=namespace,
            scale_factor=scale_factor,
            visualize_placo=visualize_placo,
            control_rate_hz=control_rate_hz,
            enable_log_data=enable_log_data,
            log_dir=log_dir,
            enable_camera=enable_camera,
            auto_enable=auto_enable,
            disable_on_shutdown=disable_on_shutdown,
        )

    controller.run()


if __name__ == "__main__":
    tyro.cli(main)
