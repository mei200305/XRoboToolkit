from geometry_msgs.msg import TwistStamped
from sensor_msgs.msg import JointState
from std_msgs.msg import Float32, Header  # For the header field

from xrobotoolkit_teleop.hardware.interface import ros2_compat as ros2

# Try to import custom message, fallback to JointState if not available
try:
    from hdas_msg.msg import motor_control
    HAS_HDAS_MSG = True
except ImportError:
    HAS_HDAS_MSG = False
    print("Warning: hdas_msg not found. Using standard JointState messages for arm control.")
    # Create a simple class to mimic motor_control structure for compatibility
    class motor_control:
        def __init__(self):
            self.header = Header()
            self.kp = []
            self.kd = []
            self.t_ff = []
            self.p_des = []
            self.v_des = []


class A1XController:
    def __init__(
        self,
        arm_control_topic: str = "/motion_control/control_arm",
        gripper_control_topic: str = "/motion_control/control_gripper",
        arm_state_topic: str = "/hdas/feedback_arm",
        rate_hz: float = 100,
        gripper_position_control: bool = False,
        use_custom_msg: bool = None,  # None means auto-detect
    ):
        # Auto-detect if custom message is available
        if use_custom_msg is None:
            use_custom_msg = HAS_HDAS_MSG
        
        self.use_custom_msg = use_custom_msg
        self.node = ros2.ensure_node("galaxea_a1x")
        
        # Use JointState if custom message not available
        if use_custom_msg:
            self.pub = self.node.create_publisher(motor_control, arm_control_topic, 1)
        else:
            self.pub = self.node.create_publisher(JointState, arm_control_topic, 1)
        
        self.gripper_position_control = gripper_position_control
        if gripper_position_control:
            self.gripper_pub = self.node.create_publisher(Float32, gripper_control_topic, 1)
        else:
            if use_custom_msg:
                self.gripper_pub = self.node.create_publisher(motor_control, gripper_control_topic, 1)
            else:
                self.gripper_pub = self.node.create_publisher(JointState, gripper_control_topic, 1)
        self.sub = self.node.create_subscription(
            JointState, arm_state_topic, self.arm_state_callback, 10
        )
        self.rate_hz = float(rate_hz)

        self.qpos = [0.0] * 6
        self.qvel = [0.0] * 6
        self.qpos_gripper = 0.0
        self.qvel_gripper = 0.0
        self.timestamp = 0.0

        # motor control parameters
        self.q_des = None
        self.v_des = [0.0] * 6
        # self.kp = [2000, 2000, 1000, 200, 200, 200]
        # self.kd = [200.0, 500.0, 500, 200, 200, 200]
        # self.kp = [200, 200, 150, 100, 100, 100]
        # self.kd = [20.0, 30.0, 10, 10, 10, 10]
        self.kp = [140, 200, 120, 80, 80, 80]
        self.kd = [10, 30, 5, 10, 10, 10]
        self.t_ff = [0.0] * 6
        
        # Initialize control message based on available message type
        if use_custom_msg:
            self.arm_ctrl_msg = motor_control()
        else:
            self.arm_ctrl_msg = JointState()
            self.arm_ctrl_msg.name = [f"joint_{i+1}" for i in range(6)]

        self.q_des_gripper = [-2.8]
        self.v_des_gripper = [0.0]
        self.kp_gripper = [1]
        self.kd_gripper = [0.05]
        self.t_ff_gripper = [0.0]
        if gripper_position_control:
            self.gripper_ctrl_msg = Float32()
        else:
            if use_custom_msg:
                self.gripper_ctrl_msg = motor_control()
            else:
                self.gripper_ctrl_msg = JointState()
                self.gripper_ctrl_msg.name = ["gripper_joint"]

    def arm_state_callback(self, msg: JointState):
        """
        Callback function to handle joint state updates.
        """
        self.qpos = list(msg.position[:6])
        self.qvel = list(msg.velocity[:6])
        self.qpos_gripper = [msg.position[6]]
        self.qvel_gripper = [msg.velocity[6]]
        if self.q_des is None:
            self.q_des = self.qpos
        self.timestamp = ros2.stamp_to_sec(msg.header.stamp)

    def publish_arm_control(self):
        """
        Publishes motor control messages to the /motion_control/control_arm topic.
        """
        if self.q_des is None:
            return

        self.arm_ctrl_msg.header = Header()
        self.arm_ctrl_msg.header.stamp = ros2.now_msg(self.node)
        
        if self.use_custom_msg:
            # Use custom motor_control message
            self.arm_ctrl_msg.header.frame_id = "base_link"
            self.arm_ctrl_msg.kp = self.kp
            self.arm_ctrl_msg.kd = self.kd
            self.arm_ctrl_msg.t_ff = self.t_ff
            self.arm_ctrl_msg.p_des = self.q_des
            self.arm_ctrl_msg.v_des = self.v_des
        else:
            # Use standard JointState message
            self.arm_ctrl_msg.position = self.q_des
            self.arm_ctrl_msg.velocity = self.v_des
            self.arm_ctrl_msg.effort = self.t_ff  # Store feedforward torque in effort field

        self.pub.publish(self.arm_ctrl_msg)

    def publish_gripper_control(self):
        if self.gripper_position_control:
            # For position control, publish Float32 message
            self.gripper_ctrl_msg.data = self.q_des_gripper[0]
        else:
            self.gripper_ctrl_msg.header = Header()
            self.gripper_ctrl_msg.header.stamp = ros2.now_msg(self.node)
            
            if self.use_custom_msg:
                # Use custom motor_control message
                self.gripper_ctrl_msg.header.frame_id = "gripper_link"
                self.gripper_ctrl_msg.kp = self.kp_gripper
                self.gripper_ctrl_msg.kd = self.kd_gripper
                self.gripper_ctrl_msg.t_ff = self.t_ff_gripper
                self.gripper_ctrl_msg.p_des = self.q_des_gripper
                self.gripper_ctrl_msg.v_des = self.v_des_gripper
            else:
                # Use standard JointState message
                self.gripper_ctrl_msg.position = self.q_des_gripper
                self.gripper_ctrl_msg.velocity = self.v_des_gripper
                self.gripper_ctrl_msg.effort = self.t_ff_gripper

        self.gripper_pub.publish(self.gripper_ctrl_msg)

    def stop(self):
        """
        Destroys the ROS publishers and subscribers.
        """
        try:
            self.node.destroy_publisher(self.pub)
            self.node.destroy_publisher(self.gripper_pub)
            self.node.destroy_subscription(self.sub)
        except Exception:
            pass


class R1LiteChassisController:
    def __init__(
        self,
        chassis_state_topic: str = "/hdas/feedback_chassis",
        chassis_control_topic: str = "/motion_target/target_speed_chassis",
        rate_hz: float = 100,
    ):
        self.node = ros2.ensure_node("galaxea_chassis")
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
        return tuple(self.chassis_vel)

    def get_velocity_command(self):
        """
        Get the current velocity command as a 3D numpy array.

        Returns:
            numpy.ndarray: 3D array containing [vx, vy, omega]
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
        self.node = ros2.ensure_node("galaxea_torso")
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
