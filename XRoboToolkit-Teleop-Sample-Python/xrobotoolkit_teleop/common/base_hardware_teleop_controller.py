from __future__ import annotations

import os
import signal
import subprocess
import threading
import time
from abc import ABC, abstractmethod
from typing import Dict

import cv2
import meshcat.transformations as tf
import numpy as np

from xrobotoolkit_teleop.common.base_teleop_controller import BaseTeleopController
from xrobotoolkit_teleop.hardware.interface.base_camera import BaseCameraInterface


class HardwareTeleopController(BaseTeleopController, ABC):
    """
    An abstract base class for hardware teleoperation controllers that consolidates
    common logic for threading, logging, and visualization.
    """

    def __init__(
        self,
        robot_urdf_path: str,
        manipulator_config: dict,
        R_headset_world: np.ndarray,
        floating_base: bool,
        scale_factor: float,
        visualize_placo: bool,
        control_rate_hz: int,
        enable_log_data: bool,
        log_dir: str,
        log_freq: float,
        enable_camera: bool,
        camera_fps: int,
        **kwargs,
    ):
        super().__init__(
            robot_urdf_path=robot_urdf_path,
            manipulator_config=manipulator_config,
            floating_base=floating_base,
            R_headset_world=R_headset_world,
            scale_factor=scale_factor,
            q_init=kwargs.get("q_init"),
            dt=1.0 / control_rate_hz,
            enable_log_data=enable_log_data,
            log_dir=log_dir,
            log_freq=log_freq,
        )

        self._start_time = 0
        self.control_rate_hz = control_rate_hz
        # Rate of _send_command; subclasses may raise it above the IK rate.
        self.command_rate_hz = getattr(self, "command_rate_hz", control_rate_hz)
        self.log_freq = log_freq
        self.visualize_placo = visualize_placo
        self.enable_camera = enable_camera
        self.camera_interface: BaseCameraInterface = None
        self.camera_fps = camera_fps

        if self.visualize_placo:
            self._init_placo_viz()

        self._prev_b_button_state = False
        self._prev_a_button_state = False
        self._is_logging = False
        self._rosbag_process = None
        self._rosbag_output_file = None
        
        # Remove DataLogger if it was created by BaseTeleopController
        # We are using ros2 bag instead
        if hasattr(self, 'data_logger'):
            delattr(self, 'data_logger')

    @abstractmethod
    def _robot_setup():
        """Initializes hardware-specific interfaces (e.g., CAN, ROS)."""
        pass

    @abstractmethod
    def _initialize_camera():
        """Initializes the specific camera interface."""
        pass

    @abstractmethod
    def _update_robot_state():
        """Reads the current robot state from hardware and updates Placo."""
        pass

    @abstractmethod
    def _send_command():
        """Sends motor commands to the hardware."""
        pass

    @abstractmethod
    def _get_robot_state_for_logging() -> Dict:
        """Returns a dictionary of robot-specific data for logging."""
        pass

    @abstractmethod
    def _shutdown_robot():
        """Performs graceful shutdown of the robot hardware."""
        pass

    @abstractmethod
    def _get_camera_frame_for_logging(self) -> Dict:
        """Returns a dictionary of camera frames for logging with camera names as keys."""
        pass

    def _get_link_pose(self, link_name: str):
        """Gets the current world pose for a given link name from Placo."""
        T_world_link = self.placo_robot.get_T_world_frame(link_name)
        pos = T_world_link[:3, 3]
        quat = tf.quaternion_from_matrix(T_world_link)
        return pos, quat

    def _get_rosbag_topics(self) -> list[str]:
        """Returns robot-specific ROS topics to record."""
        return [
            "/joint_states_single",
            "/left_piper/joint_states_single",
            "/right_piper/joint_states_single",
        ]

    def _get_rosbag_camera_topics(self) -> list[str]:
        """Returns camera ROS topics to record."""
        return [
            "/head_camera/color/image_raw",
            "/left_camera/color/image_raw",
            "/right_camera/color/image_raw",
        ]

    def _start_rosbag_recording(self):
        """Starts ros2 bag recording of selected topics."""
        if not self.enable_log_data:
            return

        # Create log directory if it doesn't exist
        os.makedirs(self.log_dir, exist_ok=True)
        
        # Generate output filename with timestamp
        timestamp = time.strftime("%Y%m%d_%H%M%S")
        self._rosbag_output_file = os.path.join(self.log_dir, f"teleop_{timestamp}")
        
        topics = self._get_rosbag_topics()
        
        # Add camera topics if camera is enabled
        if self.enable_camera:
            topics.extend(self._get_rosbag_camera_topics())
        
        # ROS 2 bag record (output is a directory)
        cmd = ["ros2", "bag", "record", "-o", self._rosbag_output_file] + topics
        
        print(f"Starting ros2 bag recording to: {self._rosbag_output_file}")
        print(f"Recording topics: {topics}")
        
        try:
            # Start ros2 bag in a subprocess
            self._rosbag_process = subprocess.Popen(
                cmd,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                preexec_fn=os.setsid  # So we can kill the whole process group
            )
            # Wait a moment to ensure recorder started
            time.sleep(0.5)
            if self._rosbag_process.poll() is not None:
                # Process died immediately
                _, stderr = self._rosbag_process.communicate()
                print(f"Failed to start ros2 bag: {stderr.decode()}")
                self._rosbag_process = None
        except Exception as e:
            print(f"Error starting ros2 bag: {e}")
            self._rosbag_process = None

    def _stop_rosbag_recording(self, save=True):
        """Stops ros2 bag recording."""
        if self._rosbag_process is None:
            return
        
        print(f"{'Stopping and saving' if save else 'Stopping and discarding'} ros2 bag recording...")
        
        try:
            if save:
                # Send SIGINT to gracefully stop bag record (it will save the file)
                os.killpg(os.getpgid(self._rosbag_process.pid), signal.SIGINT)
            else:
                # Force kill without saving
                os.killpg(os.getpgid(self._rosbag_process.pid), signal.SIGTERM)
            
            # Wait for process to finish
            self._rosbag_process.wait(timeout=5)
            if save:
                print(f"✅ Ros2 bag saved to: {self._rosbag_output_file}")
        except subprocess.TimeoutExpired:
            print("Ros2 bag did not stop gracefully, forcing termination...")
            os.killpg(os.getpgid(self._rosbag_process.pid), signal.SIGTERM)
            self._rosbag_process.wait()
        except Exception as e:
            print(f"Error stopping ros2 bag: {e}")
        finally:
            self._rosbag_process = None
            self._rosbag_output_file = None

    def _log_data(self):
        """Logs the current state of the robot and camera. (Legacy - now using ros2 bag)"""
        # Kept for backward compatibility but not used when bag recording is active
        if not self.enable_log_data:
            return

        timestamp = time.time() - self._start_time
        data_entry = {"timestamp": timestamp}
        data_entry.update(self._get_robot_state_for_logging())

        if self.enable_camera and self.camera_interface:
            frames = self._get_camera_frame_for_logging()
            if frames:
                data_entry["image"] = frames

        self.data_logger.add_entry(data_entry)

    def _pre_ik_update(self):
        """Hook for subclasses to run logic before the main IK update."""
        pass

    @staticmethod
    def _run_periodic(stop_event: threading.Event, rate_hz: float, step):
        """Call step() on a fixed deadline grid so the period does not drift with jitter."""
        period = 1.0 / rate_hz
        next_tick = time.monotonic()
        while not stop_event.is_set():
            step()
            next_tick += period
            sleep_time = next_tick - time.monotonic()
            if sleep_time > 0:
                time.sleep(sleep_time)
            else:
                # Overran: restart the grid instead of bursting to catch up.
                next_tick = time.monotonic()

    def _ik_step(self):
        self._update_robot_state()
        self._update_gripper_target()
        self._pre_ik_update()       ## 底盘和腰部
        self._update_ik()
        if self.visualize_placo:
            self._update_placo_viz()

    def _ik_thread(self, stop_event: threading.Event):
        """Dedicated thread for running the IK solver."""
        self._run_periodic(stop_event, self.control_rate_hz, self._ik_step)
        print("IK loop has stopped.")

    def _control_thread(self, stop_event: threading.Event):
        """Dedicated thread for sending commands to hardware."""
        self._run_periodic(stop_event, self.command_rate_hz, self._send_command)
        self._shutdown_robot()
        print("Control loop has stopped.")

    def _data_logging_thread(self, stop_event: threading.Event):
        """Dedicated thread for data logging. Now uses ros2 bag instead of DataLogger."""
        while not stop_event.is_set():
            start_time = time.time()
            self._check_logging_button()
            elapsed_time = time.time() - start_time
            sleep_time = (1.0 / self.log_freq) - elapsed_time
            if sleep_time > 0:
                time.sleep(sleep_time)
        print("Data logging thread has stopped.")

    def _check_logging_button(self):
        """Checks controller buttons for logging and utility actions."""
        a_button_state = self.xr_client.get_button_state_by_name("A")
        b_button_state = self.xr_client.get_button_state_by_name("B")
        right_axis_click = self.xr_client.get_button_state_by_name("right_axis_click")

        if a_button_state and not self._prev_a_button_state:
            self._handle_a_button_press()

        if b_button_state and not self._prev_b_button_state:
            if not self._is_logging:
                # Start ros2 bag recording
                self._start_rosbag_recording()
                self._is_logging = True
                if self._rosbag_process is None:
                    # Ros2 bag failed to start, revert logging state
                    self._is_logging = False
            else:
                # Stop and save ros2 bag recording
                self._stop_rosbag_recording(save=True)
                self._is_logging = False

        if right_axis_click and self._is_logging:
            # Discard current recording
            self._stop_rosbag_recording(save=False)
            self._is_logging = False

        self._prev_a_button_state = a_button_state
        self._prev_b_button_state = b_button_state

    def _handle_a_button_press(self):
        """Hook for robot-specific A-button behavior."""
        pass

    def _camera_thread(self, stop_event: threading.Event):
        """Dedicated thread for managing the camera lifecycle and streaming."""
        if not self.camera_interface:
            return

        print("Camera thread started.")
        window_name = "Hardware Cameras"
        window_created = False

        try:
            while not stop_event.is_set():
                self.camera_interface.update_frames()
                if self._is_logging:
                    if not window_created:
                        cv2.namedWindow(window_name, cv2.WINDOW_AUTOSIZE)
                        window_created = True

                    frames_dict = self.camera_interface.get_frames()
                    if not frames_dict:
                        time.sleep(1.0 / self.camera_fps)
                        continue

                    all_camera_rows = []
                    sorted_camera_names = sorted(frames_dict.keys())

                    for name in sorted_camera_names:
                        frames = frames_dict[name]
                        images_in_row = []

                        color_image = frames.get("color")
                        if color_image is not None:
                            if len(color_image.shape) == 2:
                                color_image = cv2.cvtColor(color_image, cv2.COLOR_GRAY2BGR)
                            images_in_row.append(color_image)

                        depth_image = frames.get("depth")
                        if depth_image is not None:
                            depth_colormap = cv2.applyColorMap(
                                cv2.convertScaleAbs(depth_image, alpha=0.03),
                                cv2.COLORMAP_JET,
                            )
                            images_in_row.append(depth_colormap)

                        if images_in_row:
                            all_camera_rows.append(np.hstack(images_in_row))

                    if all_camera_rows:
                        max_width = max(row.shape[1] for row in all_camera_rows)
                        padded_rows = [
                            (
                                np.hstack(
                                    [
                                        row,
                                        np.zeros(
                                            (row.shape[0], max_width - row.shape[1], 3),
                                            dtype=np.uint8,
                                        ),
                                    ]
                                )
                                if row.shape[1] < max_width
                                else row
                            )
                            for row in all_camera_rows
                        ]
                        if padded_rows:
                            combined_image = np.vstack(padded_rows)
                            cv2.imshow(
                                window_name,
                                cv2.cvtColor(combined_image, cv2.COLOR_RGB2BGR),
                            )

                    if cv2.waitKey(1) & 0xFF == ord("q"):
                        break
                else:
                    if window_created:
                        cv2.destroyWindow(window_name)
                        window_created = False
                    time.sleep(1.0 / self.camera_fps)
        finally:
            if self.camera_interface:
                self.camera_interface.stop()
            if window_created:
                cv2.destroyAllWindows()
            print("Camera thread has stopped.")

    def _should_keep_running(self) -> bool:
        """Returns True if the main loop should continue running."""
        return not self._stop_event.is_set()

    def run(self):
        """Main entry point that starts all threads."""
        self._robot_setup()
        self._initialize_camera()

        self._start_time = time.time()
        self._stop_event = threading.Event()
        threads = []

        core_threads = {
            "_ik_thread": self._ik_thread,
            "_control_thread": self._control_thread,
        }
        for name, target in core_threads.items():
            thread = threading.Thread(name=name, target=target, args=(self._stop_event,))
            threads.append(thread)

        if self.enable_log_data:
            log_thread = threading.Thread(
                name="_data_logging_thread",
                target=self._data_logging_thread,
                args=(self._stop_event,),
            )
            threads.append(log_thread)
        if self.enable_camera and self.camera_interface:
            camera_thread = threading.Thread(
                name="_camera_thread",
                target=self._camera_thread,
                args=(self._stop_event,),
            )
            threads.append(camera_thread)

        for t in threads:
            t.daemon = True
            t.start()

        print("Teleoperation running. Press Ctrl+C to exit.")
        try:
            while self._should_keep_running():
                all_threads_alive = all(t.is_alive() for t in threads)
                if not all_threads_alive:
                    print("A thread has died. Shutting down.")
                    break
                time.sleep(0.1)
        except KeyboardInterrupt:
            print("\nKeyboard interrupt received.")
        finally:
            print("Shutting down...")
            self._stop_event.set()
            # Ensure ros2 bag is stopped if still recording
            if self._is_logging and self._rosbag_process:
                self._stop_rosbag_recording(save=True)
                self._is_logging = False
            for t in threads:
                t.join(timeout=2.0)
            print("All threads have been shut down.")
