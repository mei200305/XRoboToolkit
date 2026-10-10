from geometry_msgs.msg import TwistStamped
from sensor_msgs.msg import JointState
from std_msgs.msg import Float32, Header, Float64MultiArray

from xrobotoolkit_teleop.hardware.interface import ros2_compat as ros2

# Try to import Robotiq gripper message
try:
    from robotiq_2f_gripper_control.msg import Robotiq2FGripper_robot_output
    HAS_ROBOTIQ_MSG = True
except ImportError:
    HAS_ROBOTIQ_MSG = False
    print("Warning: robot_controller/Robotiq2FGripper_robot_output message not found. Gripper control will be disabled.")
    # Create a simple class to mimic Robotiq message structure for compatibility
    class Robotiq2FGripper_robot_output:
        def __init__(self):
            self.rACT = 0  # Activate (0=reset, 1=activate)
            self.rGTO = 0  # Go to (0=stop, 1=go)
            self.rATR = 0  # Auto release (0=disable, 1=enable)
            self.rPR = 0   # Position (0=open, 255=close)
            self.rSP = 0   # Speed (0-255)
            self.rFR = 0   # Force (0-255)


class QJ2DualArmController:
    """
    Controller for QJ robot dual arm system.
    According to the ROS interface documentation:
    - Arm control: /motor_command/arm_position_control (Float64MultiArray, 28 dims)
      Format: [left7_pos, right7_pos, left7_vel, right7_vel]
    - Arm state: /motor_state/arm_motor_state_actual (JointState, 14 positions)
      Format: [left7_pos, right7_pos]
    - Gripper control: Robotiq2FGripperRobotOutput / Robotiq2FGripperRobotOutputRight
    """
    
    def __init__(
        self,
        arm_control_topic: str = "/motor_command/arm_position_control",
        left_gripper_topic: str = "Robotiq2FGripperRobotOutput",
        right_gripper_topic: str = "Robotiq2FGripperRobotOutputRight",
        arm_state_topic: str = "/motor_state/arm_motor_state_actual",
        rate_hz: float = 100,
        init_grippers: bool = True,
        gripper_init_rPR: int = 0,
        gripper_init_retries: int = 10,
        gripper_init_period_s: float = 0.1,
    ):
        self.node = ros2.ensure_node("qj2_dual_arm")

        # Publisher for arm control (28-dim Float64MultiArray)
        self.pub = self.node.create_publisher(Float64MultiArray, arm_control_topic, 1)
        
        # Publishers for gripper control
        if HAS_ROBOTIQ_MSG:
            self.left_gripper_pub = self.node.create_publisher(
                Robotiq2FGripper_robot_output, left_gripper_topic, 1
            )
            self.right_gripper_pub = self.node.create_publisher(
                Robotiq2FGripper_robot_output, right_gripper_topic, 1
            )
        else:
            self.left_gripper_pub = None
            self.right_gripper_pub = None
            print("Warning: Robotiq message not available, gripper control disabled.")
        
        # Subscriber for arm state
        self.sub = self.node.create_subscription(
            JointState, arm_state_topic, self.arm_state_callback, 10
        )
        self.rate_hz = float(rate_hz)
        
        # State variables: left arm (7 joints) + right arm (7 joints) = 14 total
        self.qpos_left = [0.0] * 7
        self.qpos_right = [0.0] * 7
        self.qvel_left = [0.0] * 7
        self.qvel_right = [0.0] * 7
        self.timestamp = 0.0
        
        # Control variables
        self.q_des_left = None
        self.q_des_right = None
        self.v_des_left = [0.0] * 7
        self.v_des_right = [0.0] * 7
        
        # Control message
        self.arm_ctrl_msg = Float64MultiArray()
        self.arm_ctrl_msg.data = [0.0] * 28  # [left7_pos, right7_pos, left7_vel, right7_vel]
        
        # Gripper control messages
        if HAS_ROBOTIQ_MSG:
            self.left_gripper_cmd = Robotiq2FGripper_robot_output()
            self.left_gripper_cmd.rACT = 1  # Activate
            self.left_gripper_cmd.rGTO = 1  # Go to
            self.left_gripper_cmd.rATR = 0  # Disable auto-release
            self.left_gripper_cmd.rSP = 255  # Max speed
            self.left_gripper_cmd.rFR = 50   # Safe force
            
            self.right_gripper_cmd = Robotiq2FGripper_robot_output()
            self.right_gripper_cmd.rACT = 1  # Activate
            self.right_gripper_cmd.rGTO = 1  # Go to
            self.right_gripper_cmd.rATR = 0  # Disable auto-release
            self.right_gripper_cmd.rSP = 255  # Max speed
            self.right_gripper_cmd.rFR = 50   # Safe force
            self._init_grippers_enabled = init_grippers
            self._gripper_init_rPR = int(max(0, min(255, gripper_init_rPR)))
            self._gripper_init_retries = max(1, int(gripper_init_retries))
            self._gripper_init_period_s = float(gripper_init_period_s)
        else:
            self._init_grippers_enabled = False
            self._gripper_init_rPR = 0
            self._gripper_init_retries = 0
            self._gripper_init_period_s = 0.0
        
        # Gripper position targets (0-255, 0=open, 255=close)
        self.left_gripper_pos = 0
        self.right_gripper_pos = 0

        # Optional: initialize Robotiq grippers on startup
        if HAS_ROBOTIQ_MSG and self._init_grippers_enabled:
            self.initialize_robotiq_grippers()

    def initialize_robotiq_grippers(self):
        """
        初始化 Robotiq 2F 夹爪：
        - 连续发布几次激活命令（rACT=1, rGTO=1）
        - 将 rPR 设置为初始位置（默认 0 = 张开）

        注意：该初始化只负责“发激活命令”，是否真正激活成功取决于你的 robotiq 驱动节点。
        """
        if not HAS_ROBOTIQ_MSG or self.left_gripper_pub is None or self.right_gripper_pub is None:
            return

        try:
            self.node.get_logger().info(
                f"[QJ2DualArmController] Initializing Robotiq grippers: "
                f"rPR={self._gripper_init_rPR}, retries={self._gripper_init_retries}, "
                f"period={self._gripper_init_period_s}s"
            )
        except Exception:
            pass

        for _ in range(self._gripper_init_retries):
            # Left
            self.left_gripper_cmd.rACT = 1
            self.left_gripper_cmd.rGTO = 1
            self.left_gripper_cmd.rATR = 0
            self.left_gripper_cmd.rPR = self._gripper_init_rPR
            self.left_gripper_pub.publish(self.left_gripper_cmd)

            # Right
            self.right_gripper_cmd.rACT = 1
            self.right_gripper_cmd.rGTO = 1
            self.right_gripper_cmd.rATR = 0
            self.right_gripper_cmd.rPR = self._gripper_init_rPR
            self.right_gripper_pub.publish(self.right_gripper_cmd)

            ros2.sleep(self._gripper_init_period_s)
    
    def arm_state_callback(self, msg: JointState):
        """
        Callback function to handle joint state updates.
        Expects 14 positions: [left7, right7]
        """
        if len(msg.position) < 14:
            self.node.get_logger().warn(
                f"Received joint state with {len(msg.position)} positions, expected 14"
            )
            return
        
        # Extract left arm (0-6) and right arm (7-13)
        self.qpos_left = list(msg.position[0:7])
        self.qpos_right = list(msg.position[7:14])
        
        # Extract velocities if available
        if len(msg.velocity) >= 14:
            self.qvel_left = list(msg.velocity[0:7])
            self.qvel_right = list(msg.velocity[7:14])
        else:
            # If velocity not provided, set to zero
            self.qvel_left = [0.0] * 7
            self.qvel_right = [0.0] * 7
        
        # Initialize desired positions if not set
        if self.q_des_left is None:
            self.q_des_left = self.qpos_left.copy()
        if self.q_des_right is None:
            self.q_des_right = self.qpos_right.copy()
        
        self.timestamp = ros2.stamp_to_sec(msg.header.stamp)
    
    def publish_arm_control(self):
        """
        Publishes motor control messages in Float64MultiArray format.
        Format: [left7_pos, right7_pos, left7_vel, right7_vel] = 28 dimensions
        """
        if self.q_des_left is None or self.q_des_right is None:
            return
        
        # Prepare 28-dim array
        # [0:7]   = left arm 7 joint positions
        # [7:14]  = right arm 7 joint positions
        # [14:21] = left arm 7 joint velocities
        # [21:28] = right arm 7 joint velocities
        #
        # 注意：self.q_des_left / right 可能是 numpy 数组，直接用 + 号会做“逐元素相加”，
        # 而不是 Python 列表拼接，所以这里先转成 list，再做拼接。
        left_pos = list(self.q_des_left)[:7]
        right_pos = list(self.q_des_right)[:7]
        left_vel = list(self.v_des_left)[:7]
        right_vel = list(self.v_des_right)[:7]

        self.arm_ctrl_msg.data = left_pos + right_pos + left_vel + right_vel
        self.pub.publish(self.arm_ctrl_msg)
    
    def publish_gripper_control(self):
        """
        Publishes gripper control messages for both left and right grippers.
        Position: 0 (open) to 255 (close)
        """
        if not HAS_ROBOTIQ_MSG or self.left_gripper_pub is None:
            return
        
        # Left gripper
        self.left_gripper_cmd.rPR = int(max(0, min(255, self.left_gripper_pos)))
        self.left_gripper_pub.publish(self.left_gripper_cmd)
        
        # Right gripper
        self.right_gripper_cmd.rPR = int(max(0, min(255, self.right_gripper_pos)))
        self.right_gripper_pub.publish(self.right_gripper_cmd)
    
    def set_gripper_position(self, left_pos: float = None, right_pos: float = None):
        """
        Set gripper position (0.0 = open, 1.0 = close).
        Converts to 0-255 range for Robotiq message.
        
        Args:
            left_pos: Left gripper position (0.0-1.0), None to keep current
            right_pos: Right gripper position (0.0-1.0), None to keep current
        """
        if left_pos is not None:
            self.left_gripper_pos = int(left_pos * 255)
        if right_pos is not None:
            self.right_gripper_pos = int(right_pos * 255)
    
    def get_left_arm_state(self):
        """Get left arm joint positions (7 joints)."""
        return self.qpos_left
    
    def get_right_arm_state(self):
        """Get right arm joint positions (7 joints)."""
        return self.qpos_right
    
    def stop(self):
        """
        Destroys the ROS publishers and subscribers.
        """
        try:
            self.node.destroy_publisher(self.pub)
            if self.left_gripper_pub:
                self.node.destroy_publisher(self.left_gripper_pub)
            if self.right_gripper_pub:
                self.node.destroy_publisher(self.right_gripper_pub)
            self.node.destroy_subscription(self.sub)
        except Exception:
            pass


# Keep old classes for backward compatibility (if needed)
class R1LiteChassisController:
    def __init__(
        self,
        chassis_state_topic: str = "/hdas/feedback_chassis",
        chassis_control_topic: str = "/motion_target/target_speed_chassis",
        rate_hz: float = 100,
    ):
        self.node = ros2.ensure_node("qj2_chassis")
        self.pub = self.node.create_publisher(TwistStamped, chassis_control_topic, 1)
        self.sub = self.node.create_subscription(
            JointState, chassis_state_topic, self.chassis_state_callback, 10
        )
        self.rate_hz = float(rate_hz)

        # Chassis state variables
        self.chassis_pos = None
        self.chassis_vel = None
        self.timestamp = -1  # Initialize with -1 to indicate no data received yet

        # Chassis control parameters
        self.twist_stamped_cmd = TwistStamped()

    def chassis_state_callback(self, msg: JointState):
        """
        Callback function to handle chassis state updates.
        """
        # Assuming the chassis state is provided as [x, y, theta] in position
        # and [vx, vy, omega] in velocity
        self.chassis_pos = msg.position
        self.chassis_vel = msg.velocity
        self.timestamp = ros2.stamp_to_sec(msg.header.stamp)

    def publish_chassis_control(self):
        """
        Publishes chassis control messages using geometry_msgs/TwistStamped.
        """
        # Set the header with timestamp and frame
        self.twist_stamped_cmd.header = Header()
        self.twist_stamped_cmd.header.stamp = ros2.now_msg(self.node)
        self.twist_stamped_cmd.header.frame_id = "base_link"

        self.pub.publish(self.twist_stamped_cmd)

    def set_velocity_command(self, vx: float, vy: float, omega: float):
        """
        Set the desired velocity command for the chassis.

        Args:
            vx: Linear velocity in x direction (m/s)
            vy: Linear velocity in y direction (m/s)
            omega: Angular velocity around z axis (rad/s)
        """
        self.twist_stamped_cmd.twist.linear.x = vx
        self.twist_stamped_cmd.twist.linear.y = vy
        self.twist_stamped_cmd.twist.linear.z = 0.0

        self.twist_stamped_cmd.twist.angular.x = 0.0
        self.twist_stamped_cmd.twist.angular.y = 0.0
        self.twist_stamped_cmd.twist.angular.z = omega

    def get_chassis_state(self):
        """
        Get the current chassis state.

        Returns:
            tuple: (position, velocity, timestamp)
        """
        return self.chassis_pos, self.chassis_vel, self.timestamp

    def get_chassis_velocity(self):
        """
        Get the current chassis velocity as a tuple.

        Returns:
            tuple: (vx, vy, omega)
        """
        return tuple(self.chassis_vel) if self.chassis_vel else (0.0, 0.0, 0.0)

    def get_velocity_command(self):
        """
        Get the current velocity command as a 3D numpy array.

        Returns:
            list: 3D array containing [vx, vy, omega]
        """
        return [
            self.twist_stamped_cmd.twist.linear.x,
            self.twist_stamped_cmd.twist.linear.y,
            self.twist_stamped_cmd.twist.angular.z,
        ]

    def stop_chassis(self):
        """
        Stop the chassis by setting all velocities to zero.
        """
        self.set_velocity_command(0.0, 0.0, 0.0)
        self.publish_chassis_control()

    def run(self):
        """
        Main loop to run the chassis controller.
        """
        period = 1.0 / self.rate_hz if self.rate_hz > 0 else 0.01
        while ros2.ok():
            self.publish_chassis_control()
            ros2.sleep(period)


class R1LiteTorsoController:
    def __init__(
        self,
        torso_state_topic: str = "/hdas/feedback_torso",
        torso_control_topic: str = "/motion_target/target_speed_torso",
        rate_hz: float = 100,
    ):
        self.node = ros2.ensure_node("qj2_torso")
        self.pub = self.node.create_publisher(TwistStamped, torso_control_topic, 1)
        self.sub = self.node.create_subscription(
            JointState, torso_state_topic, self.torso_state_callback, 10
        )
        self.rate_hz = float(rate_hz)

        # Torso state variables
        self.torso_pos = None
        self.torso_vel = None
        self.timestamp = -1
        self.twist_stamped_cmd = TwistStamped()

    def torso_state_callback(self, msg: JointState):
        """
        Callback function to handle torso state updates.
        """
        self.torso_pos = msg.position
        self.torso_vel = msg.velocity
        self.timestamp = ros2.stamp_to_sec(msg.header.stamp)

    def publish_torso_control(self):
        """
        Publishes torso control messages using geometry_msgs/TwistStamped.
        """
        self.twist_stamped_cmd.header = Header()
        self.twist_stamped_cmd.header.stamp = ros2.now_msg(self.node)
        self.twist_stamped_cmd.header.frame_id = "base_link"

        self.pub.publish(self.twist_stamped_cmd)

    def set_velocity_command(self, vz: float):
        """
        Set the desired velocity command for the torso.
        """
        self.twist_stamped_cmd.twist.linear.x = 0.0
        self.twist_stamped_cmd.twist.linear.y = 0.0
        self.twist_stamped_cmd.twist.linear.z = vz
        self.twist_stamped_cmd.twist.angular.x = 0.0
        self.twist_stamped_cmd.twist.angular.y = 0.0
        self.twist_stamped_cmd.twist.angular.z = 0.0

    def stop_torso(self):
        """
        Stop the torso by setting all velocities to zero.
        """
        self.set_velocity_command(0.0)
        self.publish_torso_control()
