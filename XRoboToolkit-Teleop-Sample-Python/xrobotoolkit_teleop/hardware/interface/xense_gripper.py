"""Xense (千觉) parallel grippers for QJ2M teleop.

Left  MAC f293c29f8d37 on 192.168.99.30 via enx00e04c2a8298 (host 192.168.99.5).
Right MAC ee82cb52277b on 192.168.98.3  via enx00e04c2c6198 (host 192.168.98.6).

Position is jaw opening in millimeters: 85 mm fully open, 0 mm fully closed.
A background thread sends set_position so a slow DDS call does not stall the arm loop.
"""
import json
import os
import sys
import threading
import time
from pathlib import Path

# Official XenseGripper.set_position range. 85 mm = open, 0 mm = closed.
XENSE_OPEN_MM = 85.0
XENSE_CLOSE_MM = 0.0
XENSE_VMAX_MM_S = 80.0
XENSE_FMAX_N = 45.0

XENSE_GRIPPERS = {
    "left_arm": {
        "mac": "f293c29f8d37",
        "ip": "192.168.99.30",
        "iface": "enx00e04c2a8298",
        # Direct visuo-tactile cameras on this gripper board.
        "tactile_sensors": ("OG001163", "OG001164"),
    },
    "right_arm": {
        "mac": "ee82cb52277b",
        "ip": "192.168.98.3",
        "iface": "enx00e04c2c6198",
        "tactile_sensors": ("OG001151", "OG001152"),
    },
}
# Four get_img calls take about 0.47 s, so 2 Hz keeps each sample aligned.
XENSE_LOG_HZ = 2.0
# Upper bound for the rosbag tactile poll; the 30 Hz frame clock publishes the latest image.
XENSE_TACTILE_POLL_HZ = 30.0
XENSE_STATUS_POLL_HZ = 50.0

# Left gripper is the "usb" side, right gripper is the "nic" side.
XENSE_GRIPPER_TOPICS = {
    "left_arm": {
        "state": "/xense/gripper/usb/state",
        "command": "/xense/gripper/usb/position_mm",
    },
    "right_arm": {
        "state": "/xense/gripper/nic/state",
        "command": "/xense/gripper/nic/position_mm",
    },
}


def tactile_image_topic(arm_name: str, serial: str) -> str:
    side = "usb" if arm_name == "left_arm" else "nic"
    return f"/xense/tactile/{side}/{serial}/image"


def xense_record_topics() -> list:
    """Gripper Float64 topics and tactile image topics recorded in one rosbag."""
    topics = []
    for spec in XENSE_GRIPPER_TOPICS.values():
        topics.extend(spec.values())
    for arm_name, cfg in XENSE_GRIPPERS.items():
        for serial in cfg["tactile_sensors"]:
            topics.append(tactile_image_topic(arm_name, serial))
    return topics

_DDS_XML = Path(__file__).with_name("xense_cyclonedds.xml")
# SDK copied from the gripper board: xensegripper, xensesdk, cyclonedds, cypack.
_XENSE_ROOT = Path(__file__).resolve().parents[4] / "third_party" / "xense"


def _ensure_xense_import_path() -> None:
    root = str(_XENSE_ROOT)
    if not _XENSE_ROOT.is_dir():
        raise FileNotFoundError(f"Xense SDK not found at {root}")
    if root not in sys.path:
        sys.path.insert(0, root)


def _ensure_cyclonedds_uri() -> None:
    """Bind DDS to both gripper NICs before xensegripper is imported."""
    if os.environ.get("CYCLONEDDS_URI"):
        return
    os.environ["CYCLONEDDS_URI"] = _DDS_XML.resolve().as_uri()


def close_fraction_to_mm(close_fraction: float) -> float:
    """Map trigger close fraction in [0, 1] to jaw opening in mm."""
    fraction = max(0.0, min(1.0, float(close_fraction)))
    opening = XENSE_OPEN_MM + (XENSE_CLOSE_MM - XENSE_OPEN_MM) * fraction
    return max(XENSE_CLOSE_MM, min(XENSE_OPEN_MM, opening))


class XenseTeleopGrippers:
    """Both Xense grippers, commanded by a close fraction per arm."""

    def __init__(self, vmin_delta_mm: float = 0.4):
        self._vmin_delta_mm = float(vmin_delta_mm)
        self._grippers = {}
        self._targets = {}
        self._last_sent = {}
        self._lock = threading.Lock()
        self._stop = threading.Event()
        self._thread = None
        self._status_thread = None
        self._current_mm = {}
        self._last_error_log = 0.0
        self._log_stop = threading.Event()
        self._log_thread = None
        self._log_dir = None

    def connect(self) -> None:
        _ensure_xense_import_path()
        _ensure_cyclonedds_uri()
        from xensegripper import XenseGripper

        for arm_name, cfg in XENSE_GRIPPERS.items():
            try:
                gripper = XenseGripper.create(cfg["mac"])
                status = gripper.get_gripper_status()
                self._grippers[arm_name] = gripper
                print(
                    f"[Xense] {arm_name} connected mac={cfg['mac']} ip={cfg['ip']} "
                    f"opening={status.get('position')} mm"
                )
            except Exception as exc:
                print(f"[Xense] {arm_name} connect failed ({cfg['ip']}): {exc}")
        if not self._grippers:
            raise RuntimeError("no Xense gripper connected")
        self._thread = threading.Thread(target=self._loop, name="xense-gripper", daemon=True)
        self._thread.start()
        self._status_thread = threading.Thread(
            target=self._status_loop, name="xense-gripper-status", daemon=True
        )
        self._status_thread.start()

    def set_close_fraction(self, arm_name: str, close_fraction: float) -> None:
        """0 = fully open, 1 = fully closed. Ignored if that side is not connected."""
        if arm_name not in self._grippers:
            return
        opening_mm = close_fraction_to_mm(close_fraction)
        with self._lock:
            self._targets[arm_name] = opening_mm

    def commanded_opening_mm(self) -> dict:
        with self._lock:
            return dict(self._last_sent)

    def pose_state(self) -> dict:
        """Measured opening and the teleop target, both in mm. 85 = open, 0 = closed.

        Measured opening comes from the status thread, so this never blocks on DDS.
        """
        with self._lock:
            targets = dict(self._targets)
            sent = dict(self._last_sent)
            current = dict(self._current_mm)
        state = {}
        for arm_name in list(self._grippers):
            target_mm = targets.get(arm_name)
            if target_mm is None:
                target_mm = sent.get(arm_name)
            state[arm_name] = {"current_mm": current.get(arm_name), "target_mm": target_mm}
        return state

    def _status_loop(self) -> None:
        period = 1.0 / XENSE_STATUS_POLL_HZ
        while not self._stop.is_set():
            started = time.time()
            for arm_name, gripper in list(self._grippers.items()):
                try:
                    status = gripper.get_gripper_status() or {}
                except Exception as exc:
                    now = time.time()
                    if now - self._last_error_log > 1.0:
                        self._last_error_log = now
                        print(f"[Xense] {arm_name} status read failed: {exc}")
                    continue
                if status.get("position") is not None:
                    with self._lock:
                        self._current_mm[arm_name] = float(status["position"])
            remain = period - (time.time() - started)
            if remain > 0:
                self._stop.wait(remain)

    def fetch_tactile_images(self) -> dict:
        """Sensor get_img frames keyed by the rosbag image topic."""
        _ensure_xense_import_path()
        from xensesdk import call_service

        frames = {}
        for arm_name, cfg in XENSE_GRIPPERS.items():
            for serial in cfg["tactile_sensors"]:
                topic = tactile_image_topic(arm_name, serial)
                try:
                    image = call_service(serial, "get_img", timeout_sec=1.0)
                except Exception as exc:
                    image = None
                    now = time.time()
                    if now - self._last_error_log > 1.0:
                        self._last_error_log = now
                        print(f"[Xense] {serial} get_img failed: {exc}")
                if image is not None:
                    frames[topic] = image
        return frames

    def start_recording(self, log_dir: str, hz: float = XENSE_LOG_HZ) -> str:
        """Record sensor images plus current/target opening while teleop runs."""
        if self._log_thread is not None:
            return self._log_dir
        stamp = time.strftime("%Y%m%d_%H%M%S")
        self._log_dir = os.path.join(log_dir, f"xense_{stamp}")
        os.makedirs(self._log_dir, exist_ok=True)
        self._log_stop.clear()
        self._log_thread = threading.Thread(
            target=self._record_loop,
            args=(float(hz),),
            name="xense-tactile-log",
            daemon=True,
        )
        self._log_thread.start()
        print(f"[Xense] recording tactile images and gripper pose to {self._log_dir}")
        return self._log_dir

    def stop_recording(self) -> None:
        self._log_stop.set()
        if self._log_thread is not None:
            self._log_thread.join(timeout=6.0)
            self._log_thread = None
        if self._log_dir:
            print(f"[Xense] tactile log saved to {self._log_dir}")

    def stop(self) -> None:
        self.stop_recording()
        self._stop.set()
        if self._thread is not None:
            self._thread.join(timeout=1.0)
        if self._status_thread is not None:
            self._status_thread.join(timeout=1.0)
        for arm_name, gripper in self._grippers.items():
            release = getattr(gripper, "release", None)
            if release is None:
                continue
            try:
                release()
            except Exception as exc:
                print(f"[Xense] {arm_name} release failed: {exc}")
        self._grippers.clear()

    def _loop(self) -> None:
        while not self._stop.is_set():
            with self._lock:
                targets = dict(self._targets)
            for arm_name, opening_mm in targets.items():
                gripper = self._grippers.get(arm_name)
                if gripper is None:
                    continue
                prev = self._last_sent.get(arm_name)
                if prev is not None and abs(prev - opening_mm) < self._vmin_delta_mm:
                    continue
                try:
                    gripper.set_position(opening_mm, XENSE_VMAX_MM_S, XENSE_FMAX_N)
                    self._last_sent[arm_name] = opening_mm
                except Exception as exc:
                    now = time.time()
                    if now - self._last_error_log > 1.0:
                        self._last_error_log = now
                        print(f"[Xense] {arm_name} set_position({opening_mm:.1f} mm) failed: {exc}")
            self._stop.wait(0.02)

    def _record_loop(self, hz: float) -> None:
        import cv2
        from xensesdk import call_service

        period = 1.0 / hz if hz > 0 else 0.5
        states_path = os.path.join(self._log_dir, "states.jsonl")
        index = 0
        with open(states_path, "a", encoding="utf-8") as states_file:
            while not self._log_stop.is_set():
                started = time.time()
                images = {}
                for arm_name, cfg in XENSE_GRIPPERS.items():
                    for serial in cfg["tactile_sensors"]:
                        key = f"{arm_name}_{serial}"
                        rel = os.path.join("tactile", key, f"{index:06d}.png")
                        abs_path = os.path.join(self._log_dir, rel)
                        os.makedirs(os.path.dirname(abs_path), exist_ok=True)
                        try:
                            image = call_service(serial, "get_img", timeout_sec=1.0)
                        except Exception as exc:
                            image = None
                            print(f"[Xense] {serial} get_img failed: {exc}")
                        if image is not None and cv2.imwrite(abs_path, image):
                            images[key] = rel
                record = {
                    "index": index,
                    "timestamp": started,
                    "grippers": self.pose_state(),
                    "tactile_images": images,
                }
                states_file.write(json.dumps(record) + "\n")
                states_file.flush()
                index += 1
                remaining = period - (time.time() - started)
                if remaining > 0:
                    self._log_stop.wait(remaining)
