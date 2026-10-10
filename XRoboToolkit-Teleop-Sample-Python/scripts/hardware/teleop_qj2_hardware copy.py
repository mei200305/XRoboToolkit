import tyro

from xrobotoolkit_teleop.hardware.qj2_teleop_controller import (
    DEFAULT_DUAL_A1X_URDF_PATH,
    DEFAULT_MANIPULATOR_CONFIG,
    QJ2TeleopController,
)


def main(
    robot_urdf_path: str = DEFAULT_DUAL_A1X_URDF_PATH,
    scale_factor: float = 1.5,
    enable_log_data: bool = True,
    visualize_placo: bool = True,
    control_rate_hz: int = 100,
    log_dir: str = "logs/qj2",
    enable_camera: bool = False,
):
    """
    Main function to run the QJ2 robot teleoperation.
    
    Args:
        robot_urdf_path: Path to the robot URDF file
        scale_factor: Scaling factor for motion (1.5 = 1.5x faster)
        enable_log_data: Enable data logging
        visualize_placo: Enable Placo visualization
        control_rate_hz: Control loop frequency in Hz
        log_dir: Directory for logging data
        enable_camera: Enable camera streaming (set to True if camera topics are available)
    """
    controller = QJ2TeleopController(
        robot_urdf_path=robot_urdf_path,
        manipulator_config=DEFAULT_MANIPULATOR_CONFIG,
        scale_factor=scale_factor,
        enable_log_data=enable_log_data,
        visualize_placo=visualize_placo,
        control_rate_hz=control_rate_hz,
        log_dir=log_dir,
        enable_camera=enable_camera,
    )
    controller.run()


if __name__ == "__main__":
    tyro.cli(main)
