"""Shared ROS 2 helpers replacing rospy patterns used in this package."""
from __future__ import annotations

import threading
import time
from typing import Optional

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

_lock = threading.Lock()
_node: Optional[Node] = None
_spin_thread: Optional[threading.Thread] = None


def ensure_node(name: str = "xrobotoolkit_teleop") -> Node:
    """Create (once) and return a process-wide ROS 2 node with a background spinner."""
    global _node, _spin_thread
    with _lock:
        if not rclpy.ok():
            rclpy.init(args=None)
        if _node is None:
            _node = rclpy.create_node(name)
            _spin_thread = threading.Thread(target=_spin, args=(_node,), daemon=True)
            _spin_thread.start()
        return _node


def _spin(node: Node) -> None:
    try:
        rclpy.spin(node)
    except Exception:
        pass


def ok() -> bool:
    return rclpy.ok()


def sleep(duration_s: float) -> None:
    time.sleep(duration_s)


def now_msg(node: Optional[Node] = None):
    n = node or ensure_node()
    return n.get_clock().now().to_msg()


def stamp_from_sec(seconds: float):
    """Wall-clock seconds (time.time() epoch, same as the node clock) to a Time msg."""
    from builtin_interfaces.msg import Time

    ns = int(round(float(seconds) * 1e9))
    return Time(sec=ns // 1_000_000_000, nanosec=ns % 1_000_000_000)


def stamp_to_sec(stamp) -> float:
    if stamp is None:
        return 0.0
    return float(stamp.sec) + float(stamp.nanosec) * 1e-9


def get_time(node: Optional[Node] = None) -> float:
    n = node or ensure_node()
    return n.get_clock().now().nanoseconds * 1e-9


def default_qos(depth: int = 10) -> QoSProfile:
    return QoSProfile(
        depth=depth,
        reliability=ReliabilityPolicy.RELIABLE,
        history=HistoryPolicy.KEEP_LAST,
    )


def latched_qos(depth: int = 1) -> QoSProfile:
    """Approximate ROS1 latch with transient-local durability when available."""
    from rclpy.qos import DurabilityPolicy

    return QoSProfile(
        depth=depth,
        reliability=ReliabilityPolicy.RELIABLE,
        history=HistoryPolicy.KEEP_LAST,
        durability=DurabilityPolicy.TRANSIENT_LOCAL,
    )


def shutdown() -> None:
    global _node, _spin_thread
    with _lock:
        if _node is not None:
            try:
                _node.destroy_node()
            except Exception:
                pass
            _node = None
        if rclpy.ok():
            try:
                rclpy.shutdown()
            except Exception:
                pass
        _spin_thread = None
