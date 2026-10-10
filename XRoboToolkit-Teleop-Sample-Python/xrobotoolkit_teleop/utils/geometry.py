from __future__ import annotations

import numpy as np
import meshcat.transformations as tf

R_HEADSET_TO_WORLD = np.array(
    [
        [0, 0, -1],
        [-1, 0, 0],
        [0, 1, 0],
    ]
)


def extract_yaw_from_quaternion(quat_wxyz: np.ndarray) -> float:
    """Extract yaw angle (rotation about Z-axis) from a quaternion [w, x, y, z].

    Assumes the quaternion is expressed in a frame where Z = up (world frame).
    Returns yaw in radians, range [-π, π].
    """
    w, x, y, z = quat_wxyz
    return np.arctan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def yaw_rotation_matrix(yaw: float) -> np.ndarray:
    """Return a 3x3 rotation matrix about the Z-axis by ``yaw`` radians.

    Positive yaw rotates counter-clockwise when looking from +Z down.
    """
    c = np.cos(yaw)
    s = np.sin(yaw)
    return np.array([
        [c, -s, 0],
        [s,  c, 0],
        [0,  0, 1],
    ])


def is_valid_quaternion(quat, tol=1e-6):
    if not isinstance(quat, (list, tuple, np.ndarray)) or len(quat) != 4:
        return False

    if not np.all(np.isfinite(quat)):
        return False

    magnitude = np.sqrt(np.sum(np.square(quat)))
    return abs(magnitude - 1.0) <= tol


def quaternion_to_angle_axis(quat: np.ndarray, eps: float = 1e-6) -> np.ndarray:
    """Converts a quaternion to an angle-axis representation.

    Args:
        quat: Quaternion in (w, x, y, z) format.
        eps: Tolerance for checking if the angle is close to zero.

    Returns:
        Angle-axis vector [ax*angle, ay*angle, az*angle].
    """
    q = np.array(quat, dtype=np.float64, copy=True)
    if q[0] < 0.0:
        q = -q

    w = q[0]
    vec_part = q[1:]

    angle = 2.0 * np.arccos(np.clip(w, -1.0, 1.0))
    if angle < eps:
        return np.zeros(3, dtype=np.float64)

    sin_half_angle = np.sin(angle / 2.0)
    if sin_half_angle < eps:
        return np.zeros(3, dtype=np.float64)

    axis = vec_part / sin_half_angle
    return axis * angle


def quat_diff_as_angle_axis(q1: np.ndarray, q2: np.ndarray, eps: float = 1e-6) -> np.ndarray:
    """Calculates the rotation from q1 to q2 as an angle-axis vector.

    This computes DeltaQ such that q2 = DeltaQ * q1.
    The result is the angle-axis representation of DeltaQ.

    Args:
        q1: Source quaternion (w, x, y, z).
        q2: Target quaternion (w, x, y, z).
        eps: Tolerance for small angle calculations.

    Returns:
        Angle-axis vector [ax*angle, ay*angle, az*angle] representing DeltaQ.
    """
    if not (is_valid_quaternion(q1, tol=eps) and is_valid_quaternion(q2, tol=eps)):
        print("Warning: Invalid quaternion input to calculate_rotation_error_as_angle_axis.")

    q1_inv = tf.quaternion_inverse(q1)      ## q1求逆
    delta_q = tf.quaternion_multiply(q2, q1_inv)  ## 求 q1到q2的旋转

    return quaternion_to_angle_axis(delta_q, eps)   # 转为角轴向量


def mirror_sagittal_delta(
    delta_xyz: np.ndarray,
    delta_rot: np.ndarray,
) -> tuple[np.ndarray, np.ndarray]:
    """Mirror motion deltas across the robot sagittal (XZ) plane.

    World frame convention after headset→world transform: +X forward, +Y left,
    +Z up. Bilateral symmetry therefore flips the lateral (Y) translation and
    the rotation components about axes lying in the sagittal plane (X and Z).

    Args:
        delta_xyz: Cartesian displacement (3,).
        delta_rot: Angle-axis orientation displacement (3,).

    Returns:
        (mirrored_xyz, mirrored_rot) with the same shapes.
    """
    mirrored_xyz = np.array(
        [delta_xyz[0], -delta_xyz[1], delta_xyz[2]], dtype=np.float64
    )
    mirrored_rot = np.array(
        [-delta_rot[0], delta_rot[1], -delta_rot[2]], dtype=np.float64
    )
    return mirrored_xyz, mirrored_rot


def relative_pose_6d(
    leader_xyz: np.ndarray,
    leader_quat: np.ndarray,
    follower_xyz: np.ndarray,
    follower_quat: np.ndarray,
) -> tuple[np.ndarray, np.ndarray]:
    """6D offset of follower EE relative to leader EE.

    Position is a constant world-frame translation. Orientation is the body-fixed
    relative quaternion ``q_rel`` satisfying ``q_follower = q_leader * q_rel``.
    Together they preserve the captured 6D difference while the leader moves:
    the follower copies the leader's world translation and incremental rotation.

    Args:
        leader_xyz: Leader EE position (3,).
        leader_quat: Leader EE quaternion (w, x, y, z).
        follower_xyz: Follower EE position (3,).
        follower_quat: Follower EE quaternion (w, x, y, z).

    Returns:
        (delta_xyz_world, delta_quat_wxyz).
    """
    delta_xyz = np.asarray(follower_xyz, dtype=np.float64).reshape(3) - np.asarray(
        leader_xyz, dtype=np.float64
    ).reshape(3)
    q_leader_inv = tf.quaternion_inverse(np.asarray(leader_quat, dtype=np.float64).reshape(4))
    delta_quat = tf.quaternion_multiply(
        q_leader_inv, np.asarray(follower_quat, dtype=np.float64).reshape(4)
    )
    return delta_xyz, delta_quat


def apply_relative_pose_6d(
    leader_xyz: np.ndarray,
    leader_quat: np.ndarray,
    delta_xyz: np.ndarray,
    delta_quat: np.ndarray,
) -> tuple[np.ndarray, np.ndarray]:
    """Reconstruct follower EE pose from leader pose and a captured 6D offset."""
    follower_xyz = np.asarray(leader_xyz, dtype=np.float64).reshape(3) + np.asarray(
        delta_xyz, dtype=np.float64
    ).reshape(3)
    follower_quat = tf.quaternion_multiply(
        np.asarray(leader_quat, dtype=np.float64).reshape(4),
        np.asarray(delta_quat, dtype=np.float64).reshape(4),
    )
    return follower_xyz, follower_quat


def apply_delta_pose(
    source_pos: np.ndarray,
    source_rot: np.ndarray,
    delta_pos: np.ndarray,
    delta_rot: np.ndarray,
    eps: float = 1.0e-6,
) -> tuple[np.ndarray, np.ndarray]:
    """Applies delta pose transformation on source pose using NumPy.

    Args:
        source_pos: Position of source frame. Shape is (3,).
        source_rot: Quaternion orientation of source frame in (w, x, y, z). Shape is (4,).
        delta_pos: Cartesian position displacement. Shape is (3,).
        delta_rot: Orientation displacement in angle-axis format. Shape is (3,).
        eps: The tolerance to consider orientation displacement as zero. Defaults to 1.0e-6.

    Returns:
        A tuple containing the displaced position and orientation frames.
        target_pos: Shape is (3,).
        target_rot: Shape is (4,).
    """
    if not (
        isinstance(source_pos, np.ndarray)
        and source_pos.shape == (3,)
        and isinstance(source_rot, np.ndarray)
        and source_rot.shape == (4,)
        and isinstance(delta_pos, np.ndarray)
        and delta_pos.shape == (3,)
        and isinstance(delta_rot, np.ndarray)
        and delta_rot.shape == (3,)
    ):
        raise ValueError(
            "Inputs must be 1D NumPy arrays with shapes: "
            "source_pos (3,), source_rot (4,), delta_pos (3,), delta_rot (3,)."
        )

    # Calculate target position
    target_pos = source_pos + delta_pos

    # Calculate target rotation
    angle = np.linalg.norm(delta_rot)
    rot_delta_quat: np.ndarray
    if angle > eps:
        axis = delta_rot / angle
        rot_delta_quat = tf.quaternion_about_axis(angle, axis)
    else:
        rot_delta_quat = np.array([1.0, 0.0, 0.0, 0.0])

    target_rot = tf.quaternion_multiply(rot_delta_quat, source_rot)

    return target_pos, target_rot
