"""QJ2M dual-arm ready pose and sequential left-then-right homing."""
from __future__ import annotations

import numpy as np

# Final home: measured / WBC-ready asymmetric pose used by one-key homing only.
# HOME_Q_LEFT = np.array(
#     [-0.4503316879272461, 0.6570920944213867, -1.6172657012939453, -2.0571069717407227, 0.053978919982910156, -0.3145265579223633, 0.3397035598754883]
# )
HOME_Q_LEFT = np.array(
    [0.52, 0.3, 0.0, -2.09, 0.0, 0.0, 0.0]
)

# HOME_Q_RIGHT = np.array(
#     [-0.3709850311279297, -0.3454265594482422, 1.3292512893676758, -1.7034788131713867, -0.3431367874145508, -0.9672307968139648, 0.4659719467163086]
# )
HOME_Q_RIGHT = np.array(
    [ 0.52, -0.3, 0.0, -2.09, 0.0, 0.0, 0.0]
)

# Measured dual-arm pose for left-stick-down (simultaneous interp, both arms).
STOW_Q_LEFT = np.array(
    [0.4773995053342168, 0.3090463561739181, 0.11599416982854659, -2.0687279809329233, 2.89849062347926, 0.3233899561599878, 1.2663162544028657]
)
STOW_Q_RIGHT = np.array(
    [0.5347155342631686, -0.28189585503236053, -0.02619788583484213, -2.046979553502805, -3.038031950711785, 0.3690418607352969, -0.9862807984573103]
)

# Dual-arm pose for left-stick-right phase 0 (before CMD, 4s).
CMD0_Q_LEFT = np.array(
    [-1.4222217809507078, 0.7189092475788248, -2.724505933681119, -1.9490327550003705, 2.900514998081701, 0.8219521210661519, -0.9732162656372265]
)
CMD0_Q_RIGHT = np.array(
    [-1.657723508368619, -0.19905700831771272, 3.0623616897337773, -2.026152062263037, -3.086070368294336, 0.7347002281555509, 1.1812497963541007]
)

# Dual-arm pose for left-stick-right phase 1 (simultaneous interp).
CMD_Q_LEFT = np.array(
    [-2.3915888200188666, 0.3922006678687161, -3.1105941579575176, -2.0943951023931953, 3.1008256789723196, 1.396403252481762, -1.2982452267491065]
)
CMD_Q_RIGHT = np.array(
    [-2.458634496192629, -0.07227226750943257, 3.14, -2.017374824012111, -3.036633346156976, 1.5309884783608336, 1.4]
)

# Dual-arm pose for left-stick-right phase 2 (after CMD, another 5s).
CMD2_Q_LEFT = np.array(
    [-2.1323308043823106, 0.08702998882743575, -3.1318039604062857, -1.0701034821482285, 3.14, 1.91, -1.4]
)
CMD2_Q_RIGHT = np.array(
    [-2.0244186296113065, -0.20504106637403682, 2.581078366594091, -1.1045103117337223, -3.023120388847655, 1.91, 1.3435132312256826]
)

# Online IK soft preference (null-space): ideal symmetric WBC arm_teleoperation_default_pos.
# Distinct from HOME_Q_* — homing still goes to HOME; teleop IK gently pulls toward PREF.
PREF_Q = np.array([0.52, 0.0, 0.0, -2.09, 0.0, 0.0, 0.0])
PREF_Q_LEFT = PREF_Q.copy()
PREF_Q_RIGHT = PREF_Q.copy()
JOINTS_TASK_WEIGHT = 5e-4  # soft; << EE(1.0), below manipulability(1e-2)

HOMING_DURATION = 6.0  # seconds total (left then right; keep per-arm speed)
STOW_DURATION = 5.0  # seconds — left stick down, both arms simultaneous
CMD0_DURATION = 4.0  # seconds — left stick right phase 0 → CMD0
CMD_DURATION = 4.0  # seconds — left stick right phase 1 → CMD
CMD2_DURATION = 5.0  # seconds — left stick right phase 2 → CMD2
HOMING_LEFT_FRACTION = 0.5  # first half → left arm, second half → right arm
HOMING_VIA_FRACTION = 0.5  # within each arm: J1–J3 then J4–J7


def pref_joints_dict(left_joint_names, right_joint_names, pref_left=None, pref_right=None):
    """Build Placo joints_task target dict from preferred arm postures."""
    if pref_left is None:
        pref_left = PREF_Q_LEFT
    if pref_right is None:
        pref_right = PREF_Q_RIGHT
    pref_left = np.asarray(pref_left, dtype=float).reshape(-1)
    pref_right = np.asarray(pref_right, dtype=float).reshape(-1)
    out = {}
    for name, q in zip(left_joint_names, pref_left):
        out[name] = float(q)
    for name, q in zip(right_joint_names, pref_right):
        out[name] = float(q)
    return out


def ease_in_out_cubic(t: float) -> float:
    t = max(0.0, min(1.0, t))
    return 4.0 * t * t * t if t < 0.5 else 1.0 - (-2.0 * t + 2.0) ** 3 / 2.0


def shortest_angular_path(q_from, q_to):
    """Shortest path per joint (wrap to [-π, π])."""
    diff = np.asarray(q_to, dtype=float) - np.asarray(q_from, dtype=float)
    diff = np.arctan2(np.sin(diff), np.cos(diff))
    return np.asarray(q_from, dtype=float) + diff


def shortest_angular_path_safe(q_from, q_to, lower, upper):
    """Shortest angular path constrained to joint limits."""
    q_from = np.asarray(q_from, dtype=float)
    q_to = np.asarray(q_to, dtype=float)
    lower = np.asarray(lower, dtype=float)
    upper = np.asarray(upper, dtype=float)

    diff = q_to - q_from
    diff = np.arctan2(np.sin(diff), np.cos(diff))
    result = q_from + diff

    for i in range(len(result)):
        if result[i] < lower[i]:
            candidate = result[i] + 2.0 * np.pi
            result[i] = candidate if candidate <= upper[i] else lower[i]
        elif result[i] > upper[i]:
            candidate = result[i] - 2.0 * np.pi
            result[i] = candidate if candidate >= lower[i] else upper[i]
    return result


def proximal_first_via(q0, home):
    """Via pose: J1–J3 at home, J4–J7 stay at current q0."""
    via = np.asarray(q0, dtype=float).copy()
    home = np.asarray(home, dtype=float)
    via[:3] = home[:3]
    return via


def build_homing_targets(q0_left, q0_right, home_left=None, home_right=None, path_fn=None):
    """Return (via_l, via_r, home_l, home_r): J1–J3 via, then full home per arm."""
    if home_left is None:
        home_left = HOME_Q_LEFT
    if home_right is None:
        home_right = HOME_Q_RIGHT
    if path_fn is None:
        path_fn = shortest_angular_path

    via_l = path_fn(q0_left, proximal_first_via(q0_left, home_left))
    via_r = path_fn(q0_right, proximal_first_via(q0_right, home_right))
    home_l = path_fn(via_l, home_left)
    home_r = path_fn(via_r, home_right)
    return via_l, via_r, home_l, home_r


def sample_two_phase_homing(q0, via, home, t, via_fraction=HOMING_VIA_FRACTION):
    """Interpolate q along q0→via→home. t in [0, 1]."""
    t = max(0.0, min(1.0, float(t)))
    vf = float(via_fraction)
    q0 = np.asarray(q0, dtype=float)
    via = np.asarray(via, dtype=float)
    home = np.asarray(home, dtype=float)
    if t < vf:
        s = ease_in_out_cubic(t / vf)
        return q0 + s * (via - q0)
    s = ease_in_out_cubic((t - vf) / (1.0 - vf))
    return via + s * (home - via)


def sample_simultaneous_arms(q0_left, target_left, q0_right, target_right, t):
    """Both arms ease-in-out from q0 → target in parallel. t in [0, 1]."""
    t = max(0.0, min(1.0, float(t)))
    s = ease_in_out_cubic(t)
    q0_left = np.asarray(q0_left, dtype=float)
    q0_right = np.asarray(q0_right, dtype=float)
    target_left = np.asarray(target_left, dtype=float)
    target_right = np.asarray(target_right, dtype=float)
    ql = q0_left + s * (target_left - q0_left)
    qr = q0_right + s * (target_right - q0_right)
    return ql, qr


def sample_simultaneous_two_waypoint(
    q0_left,
    via_left,
    target_left,
    q0_right,
    via_right,
    target_right,
    t,
    via_fraction=0.5,
):
    """Both arms in parallel: q0 → via → target. t in [0, 1]."""
    ql = sample_two_phase_homing(q0_left, via_left, target_left, t, via_fraction)
    qr = sample_two_phase_homing(q0_right, via_right, target_right, t, via_fraction)
    return ql, qr


def sample_three_phase(q0, w1, w2, w3, t, f1, f2):
    """Interpolate q along q0→w1→w2→w3. t in [0,1]; f1/f2 are phase fractions."""
    t = max(0.0, min(1.0, float(t)))
    f1 = max(0.0, float(f1))
    f2 = max(0.0, float(f2))
    f3 = max(1e-9, 1.0 - f1 - f2)
    q0 = np.asarray(q0, dtype=float)
    w1 = np.asarray(w1, dtype=float)
    w2 = np.asarray(w2, dtype=float)
    w3 = np.asarray(w3, dtype=float)
    if t < f1:
        s = ease_in_out_cubic(t / f1 if f1 > 0 else 1.0)
        return q0 + s * (w1 - q0)
    if t < f1 + f2:
        s = ease_in_out_cubic((t - f1) / f2 if f2 > 0 else 1.0)
        return w1 + s * (w2 - w1)
    s = ease_in_out_cubic((t - f1 - f2) / f3)
    return w2 + s * (w3 - w2)


def sample_simultaneous_three_waypoint(
    q0_left,
    w1_left,
    w2_left,
    w3_left,
    q0_right,
    w1_right,
    w2_right,
    w3_right,
    t,
    f1=1.0 / 3.0,
    f2=1.0 / 3.0,
):
    """Both arms in parallel: q0 → w1 → w2 → w3. t in [0, 1]."""
    ql = sample_three_phase(q0_left, w1_left, w2_left, w3_left, t, f1, f2)
    qr = sample_three_phase(q0_right, w1_right, w2_right, w3_right, t, f1, f2)
    return ql, qr


def sample_left_then_right_homing(
    q0_left,
    via_left,
    home_left,
    q0_right,
    via_right,
    home_right,
    t,
    left_fraction=HOMING_LEFT_FRACTION,
    via_fraction=HOMING_VIA_FRACTION,
):
    """Left arm homes first (holding right), then right arm (holding left)."""
    t = max(0.0, min(1.0, float(t)))
    lf = float(left_fraction)
    q0_left = np.asarray(q0_left, dtype=float)
    q0_right = np.asarray(q0_right, dtype=float)
    home_left = np.asarray(home_left, dtype=float)
    home_right = np.asarray(home_right, dtype=float)

    if t < lf:
        t_local = t / lf if lf > 0 else 1.0
        ql = sample_two_phase_homing(q0_left, via_left, home_left, t_local, via_fraction)
        qr = q0_right.copy()
    else:
        t_local = (t - lf) / (1.0 - lf) if lf < 1.0 else 1.0
        ql = home_left.copy()
        qr = sample_two_phase_homing(q0_right, via_right, home_right, t_local, via_fraction)
    return ql, qr
