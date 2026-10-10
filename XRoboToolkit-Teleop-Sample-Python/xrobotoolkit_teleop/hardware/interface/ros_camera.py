import threading

import cv2
import numpy as np
from cv_bridge import CvBridge
from sensor_msgs.msg import CompressedImage, Image

from ...utils.image_utils import compress_image_to_jpg
from . import ros2_compat as ros2
from .base_camera import BaseCameraInterface


class RosCameraInterface(BaseCameraInterface):
    """
    An interface to handle one or more cameras through ROS 2 topics.
    """

    def __init__(
        self,
        camera_topics: dict,
        enable_depth: bool = True,
        width: int = None,
        height: int = None,
        enable_compression: bool = True,
        jpg_quality: int = 85,
    ):
        super().__init__(enable_compression=enable_compression, jpg_quality=jpg_quality)
        self.camera_topics = camera_topics
        self.enable_depth = enable_depth
        self.width = width
        self.height = height
        self.bridge = CvBridge()
        self.node = ros2.ensure_node("ros_camera_interface")

        self.frames_dict = {}
        self.compressed_frames_dict = {}
        self.frames_lock = threading.Lock()
        self.subscribers = []

    def start(self):
        """Starts the camera subscribers."""
        for name, topics in self.camera_topics.items():
            if "color" not in topics:
                print(f"Warning: 'color' topic not specified for camera {name}. Skipping.")
                continue

            color_topic = topics["color"]
            self.subscribers.append(
                self.node.create_subscription(
                    CompressedImage,
                    color_topic,
                    lambda msg, camera_name=name: self._color_callback(msg, camera_name),
                    10,
                )
            )

            if self.enable_depth and "depth" in topics:
                depth_topic = topics["depth"]
                self.subscribers.append(
                    self.node.create_subscription(
                        Image,
                        depth_topic,
                        lambda msg, camera_name=name: self._depth_callback(msg, camera_name),
                        10,
                    )
                )
            print(f"Subscribed to topics for camera: {name}")

    def _resize_image(self, image):
        if self.width is not None and self.height is not None and image is not None:
            return cv2.resize(image, (self.width, self.height))
        return image

    def _color_callback(self, msg, camera_name):
        np_arr = np.frombuffer(msg.data, np.uint8)
        color_image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        color_image = self._resize_image(color_image)

        with self.frames_lock:
            if camera_name not in self.frames_dict:
                self.frames_dict[camera_name] = {}
                self.compressed_frames_dict[camera_name] = {}

            self.frames_dict[camera_name]["color"] = color_image

            if self.enable_compression:
                compressed_bytes = compress_image_to_jpg(color_image, self.jpg_quality)
                self.compressed_frames_dict[camera_name]["color"] = compressed_bytes

    def _depth_callback(self, msg, camera_name):
        depth_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
        depth_image = self._resize_image(depth_image)

        with self.frames_lock:
            if camera_name not in self.frames_dict:
                self.frames_dict[camera_name] = {}
                self.compressed_frames_dict[camera_name] = {}

            self.frames_dict[camera_name]["depth"] = depth_image

            if self.enable_compression:
                compressed_bytes = compress_image_to_jpg(depth_image, self.jpg_quality)
                self.compressed_frames_dict[camera_name]["depth"] = compressed_bytes

    def stop(self):
        """Stops the camera subscribers."""
        for sub in self.subscribers:
            try:
                self.node.destroy_subscription(sub)
            except Exception:
                pass
        self.subscribers.clear()
        print("Unregistered all camera subscribers.")

    def update_frames(self):
        pass

    def get_frames(self):
        with self.frames_lock:
            frames_dict = {}
            for camera_name, frame_data in self.frames_dict.items():
                color_frame = frame_data.get("color")
                depth_frame = frame_data.get("depth")
                frames_dict[camera_name] = {
                    "color": color_frame.copy() if color_frame is not None else None,
                    "depth": depth_frame.copy() if self.enable_depth and depth_frame is not None else None,
                }
            return frames_dict

    def get_compressed_frames(self):
        with self.frames_lock:
            compressed_dict = {}
            for camera_name, frame_data in self.compressed_frames_dict.items():
                color_bytes = frame_data.get("color")
                depth_bytes = frame_data.get("depth")
                compressed_dict[camera_name] = {
                    "color": color_bytes[:],
                    "depth": depth_bytes[:] if self.enable_depth else None,
                }
            return compressed_dict

    def get_frame(self, camera_name: str):
        with self.frames_lock:
            return self.frames_dict.get(camera_name, {}).copy()
