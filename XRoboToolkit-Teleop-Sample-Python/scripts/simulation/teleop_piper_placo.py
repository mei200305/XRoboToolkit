import tyro

from xrobotoolkit_teleop.hardware.piper_teleop_controller import (
    DEFAULT_PIPER_URDF_PATH,
)
from xrobotoolkit_teleop.simulation.placo_teleop_controller import (
    PlacoTeleopController,
)
from xrobotoolkit_teleop.utils.path_utils import resolve_ros_package_uris_in_urdf


def main(
    robot_urdf_path: str = DEFAULT_PIPER_URDF_PATH,
    link_name: str = "link6",
    scale_factor: float = 1.2,
    control_mode: str = "pose",
):
    """
    Run a Piper Placo-only teleoperation demo.

    This script does not require the real robot driver. It only loads the URDF,
    runs IK in Placo, and visualizes the result.
    """
    config = {
        "right_arm": {
            "link_name": link_name,
            "pose_source": "right_controller",
            "control_trigger": "right_grip",
            "control_mode": control_mode,
            "gripper_config": {
                "type": "parallel",
                "gripper_trigger": "right_trigger",
                "joint_names": ["joint7"],
                "open_pos": [0.0],
                "close_pos": [0.08],
            },
        },
    }

    resolved_urdf_path = resolve_ros_package_uris_in_urdf(
        robot_urdf_path,
        package_map={"piper_description": "/home/piper_ros/src/piper_description"},
    )

    controller = PlacoTeleopController(
        robot_urdf_path=resolved_urdf_path,
        manipulator_config=config,
        floating_base=False,
        scale_factor=scale_factor,
        dt=0.01,
    )

    joints_task = controller.solver.add_joints_task()
    joints_task.set_joints({joint: 0.0 for joint in controller.placo_robot.joint_names()})
    joints_task.configure("joints_regularization", "soft", 1e-4)

    print("=" * 60)
    print("Piper Placo simulation started")
    print("=" * 60)
    print(f"URDF: {robot_urdf_path}")
    print(f"Resolved URDF: {resolved_urdf_path}")
    print(f"End effector link: {link_name}")
    print(f"Control mode: {control_mode}")
    print("右手控制器 -> Piper 末端")
    print("按住 right grip 激活控制")
    print("right trigger 用于模拟夹爪目标")
    print("按 Ctrl+C 退出")
    print("=" * 60)

    controller.run()


if __name__ == "__main__":
    tyro.cli(main)
