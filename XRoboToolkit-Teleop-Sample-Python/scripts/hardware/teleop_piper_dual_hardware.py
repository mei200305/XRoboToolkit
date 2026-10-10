import tyro

from xrobotoolkit_teleop.hardware.piper_dual_teleop_controller import PiperDualTeleopController
from xrobotoolkit_teleop.hardware.piper_teleop_controller import DEFAULT_PIPER_URDF_PATH
from xrobotoolkit_teleop.hardware.piper_sdk_dual_teleop_controller import (
    DEFAULT_DUAL_PIPER_MANIPULATOR_CONFIG,
    PiperSDKDualTeleopController,
)


def main(
    backend: str = "sdk",
    single_arm_urdf_path: str = DEFAULT_PIPER_URDF_PATH,
    scale_factor: float = 1.0,
    enable_log_data: bool = True,
    enable_camera: bool = False,
    visualize_placo: bool = False,
    control_rate_hz: int = 100,
    log_dir: str = "logs/piper_dual",
    arm_spacing: float = 0.70,
    left_can_name: str = "can0",
    right_can_name: str = "can1",
    left_namespace: str = "/left_piper",
    right_namespace: str = "/right_piper",
    judge_flag: bool = True,
    auto_enable: bool = True,
):
    """
    Run dual-arm Piper hardware teleoperation.

    `backend="sdk"` controls both arms through piper_sdk/CAN and publishes
    logging JointState topics for rosbag.
    `backend="ros"` controls both arms through namespaced piper_ros topics.
    """
    backend = backend.lower().strip()
    if backend not in {"sdk", "ros"}:
        raise ValueError(f"Unsupported backend: {backend}. Expected 'sdk' or 'ros'.")

    print("=" * 60)
    print("Piper dual-arm hardware teleoperation")
    print("=" * 60)
    print(f"Backend: {backend}")
    print(f"Single-arm URDF: {single_arm_urdf_path}")
    print(f"Control rate: {control_rate_hz} Hz")
    print(f"Arm spacing: {arm_spacing} m")
    print(f"Record camera topics: {enable_camera}")
    print(f"Left namespace: {left_namespace}")
    print(f"Right namespace: {right_namespace}")
    if backend == "sdk":
        print(f"Left CAN interface: {left_can_name}")
        print(f"Right CAN interface: {right_can_name}")
        print(f"Judge flag: {judge_flag}")
    print("左手柄 -> 左 Piper，右手柄 -> 右 Piper")
    print("按住 left/right grip 激活对应机械臂")
    print("left/right trigger 控制对应夹爪开合")
    print("按 B 键开始/结束 rosbag 录制")
    print("按 Ctrl+C 退出")
    print("=" * 60)

    if backend == "sdk":
        controller = PiperSDKDualTeleopController(
            single_arm_urdf_path=single_arm_urdf_path,
            manipulator_config=DEFAULT_DUAL_PIPER_MANIPULATOR_CONFIG,
            scale_factor=scale_factor,
            enable_log_data=enable_log_data,
            visualize_placo=visualize_placo,
            control_rate_hz=control_rate_hz,
            log_dir=log_dir,
            arm_spacing=arm_spacing,
            left_can_name=left_can_name,
            right_can_name=right_can_name,
            left_namespace=left_namespace,
            right_namespace=right_namespace,
            judge_flag=judge_flag,
            enable_camera=enable_camera,
            auto_enable=auto_enable,
        )
    else:
        controller = PiperDualTeleopController(
            single_arm_urdf_path=single_arm_urdf_path,
            manipulator_config=DEFAULT_DUAL_PIPER_MANIPULATOR_CONFIG,
            left_namespace=left_namespace,
            right_namespace=right_namespace,
            scale_factor=scale_factor,
            enable_log_data=enable_log_data,
            visualize_placo=visualize_placo,
            control_rate_hz=control_rate_hz,
            log_dir=log_dir,
            arm_spacing=arm_spacing,
            enable_camera=enable_camera,
            auto_enable=auto_enable,
        )

    controller.run()


if __name__ == "__main__":
    tyro.cli(main)
