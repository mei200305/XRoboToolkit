import tyro

from xrobotoolkit_teleop.hardware.piper_teleop_controller import (
    DEFAULT_PIPER_URDF_PATH,
)
from xrobotoolkit_teleop.simulation.placo_teleop_controller import (
    PlacoTeleopController,
)
from xrobotoolkit_teleop.utils.path_utils import build_dual_arm_urdf_from_single


def main(
    robot_urdf_path: str = DEFAULT_PIPER_URDF_PATH,
    left_link_name: str = "left_link6",
    right_link_name: str = "right_link6",
    scale_factor: float = 1.2,
    arm_spacing: float = 0.70,
    control_mode: str = "pose",
):
    dual_urdf_path = build_dual_arm_urdf_from_single(
        robot_urdf_path,
        left_prefix="left_",
        right_prefix="right_",
        left_mount_xyz=f"0 {arm_spacing / 2.0:.4f} 0",
        right_mount_xyz=f"0 {-arm_spacing / 2.0:.4f} 0",
    )

    config = {
        "right_arm": {
            "link_name": right_link_name,
            "pose_source": "right_controller",
            "control_trigger": "right_grip",
            "control_mode": control_mode,
            "gripper_config": {
                "type": "parallel",
                "gripper_trigger": "right_trigger",
                "joint_names": ["right_joint7"],
                "open_pos": [0.0],
                "close_pos": [0.035],
            },
        },
        "left_arm": {
            "link_name": left_link_name,
            "pose_source": "left_controller",
            "control_trigger": "left_grip",
            "control_mode": control_mode,
            "gripper_config": {
                "type": "parallel",
                "gripper_trigger": "left_trigger",
                "joint_names": ["left_joint7"],
                "open_pos": [0.0],
                "close_pos": [0.035],
            },
        },
    }

    controller = PlacoTeleopController(
        robot_urdf_path=dual_urdf_path,
        manipulator_config=config,
        floating_base=False,
        scale_factor=scale_factor,
        dt=0.01,
    )

    joints_task = controller.solver.add_joints_task()
    joints_task.set_joints({joint: 0.0 for joint in controller.placo_robot.joint_names()})
    joints_task.configure("joints_regularization", "soft", 1e-4)

    print("=" * 60)
    print("Dual Piper Placo simulation started")
    print("=" * 60)
    print(f"Source URDF: {robot_urdf_path}")
    print(f"Generated dual URDF: {dual_urdf_path}")
    print(f"Arm spacing: {arm_spacing}")
    print(f"Left EE link: {left_link_name}")
    print(f"Right EE link: {right_link_name}")
    print("左手控制器 -> left Piper")
    print("右手控制器 -> right Piper")
    print("按住对应 grip 激活对应手臂")
    print("对应 trigger 模拟夹爪")
    print("按 Ctrl+C 退出")
    print("=" * 60)

    controller.run()


if __name__ == "__main__":
    tyro.cli(main)
