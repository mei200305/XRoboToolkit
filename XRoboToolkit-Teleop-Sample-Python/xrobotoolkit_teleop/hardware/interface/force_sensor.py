"""Wrist 6-axis force sensors, same serial protocol as Force.cpp.

Force.cpp uses /dev/ttyUSB1 for the left arm and /dev/ttyUSB2 for the right.
On this Jetson the adapters stay on fixed USB ports: left 1-4.4.4 (or 1-4.4.3), right 1-4.3.2.
Packets are 30 bytes starting with 0xAA 0x55. Six little-endian floats are
Fx, Fy, Fz, Tx, Ty, Tz. Each sensor is configured at 100 Hz, unit MVPV.
The /force mirror is the right-arm sample, matching Force.cpp.
"""
import math
import os
import struct
import threading
import time

LEFT_ARM_FORCE_PORT = "/dev/ttyUSB1"
LEFT_ARM_FORCE_TOPIC = "/force/left_arm"
LEFT_ARM_FORCE_FILTERED_TOPIC = "/wrench_filtered_left"
LEFT_ARM_FORCE_FRAME = "l_arm_force_link"
# This board is the CH340 on hub 1-4.4. Port 1-4.3.4 failed to enumerate.
LEFT_ARM_USB = ("1-4.4.4", "1-4.4.3")

RIGHT_ARM_FORCE_PORT = "/dev/ttyUSB2"
RIGHT_ARM_FORCE_TOPIC = "/force/right_arm"
RIGHT_ARM_FORCE_FILTERED_TOPIC = "/wrench_filtered_right"
RIGHT_ARM_FORCE_MIRROR_TOPIC = "/force"
RIGHT_ARM_FORCE_FRAME = "r_arm_force_link"
RIGHT_ARM_FORCE_MIRROR_FRAME = "force_link"
# The right-arm CH340 enumerated on USB port 1-4.3.2.
RIGHT_ARM_USB = "1-4.3.2"

FORCE_ARMS = {
    "left": {
        "usb": LEFT_ARM_USB,
        "port": LEFT_ARM_FORCE_PORT,
        "topic": LEFT_ARM_FORCE_TOPIC,
        "filtered": LEFT_ARM_FORCE_FILTERED_TOPIC,
        "frame": LEFT_ARM_FORCE_FRAME,
    },
    "right": {
        "usb": RIGHT_ARM_USB,
        "port": RIGHT_ARM_FORCE_PORT,
        "topic": RIGHT_ARM_FORCE_TOPIC,
        "filtered": RIGHT_ARM_FORCE_FILTERED_TOPIC,
        "frame": RIGHT_ARM_FORCE_FRAME,
        "mirror": RIGHT_ARM_FORCE_MIRROR_TOPIC,
        "mirror_frame": RIGHT_ARM_FORCE_MIRROR_FRAME,
    },
}

FORCE_SAMPLE_HZ = 100
FORCE_CUTOFF_HZ = 10.0
# A sensor that misses AT+GSD (e.g. still booting after a replug) stays silent forever.
FORCE_STALL_TIMEOUT_S = 2.0
_PACKET_LEN = 30
_FX_MIN, _FX_MAX = -600.0, 600.0
_T_MIN, _T_MAX = -20.0, 20.0


def force_record_topics() -> list:
    topics = []
    for spec in FORCE_ARMS.values():
        topics.append(spec["topic"])
        topics.append(spec["filtered"])
        mirror = spec.get("mirror")
        if mirror:
            topics.append(mirror)
    return topics


def right_arm_force_topics() -> list:
    return [
        RIGHT_ARM_FORCE_TOPIC,
        RIGHT_ARM_FORCE_FILTERED_TOPIC,
        RIGHT_ARM_FORCE_MIRROR_TOPIC,
    ]


def round_half_away(value: float) -> float:
    """Match std::llround used when Force.cpp publishes a wrench."""
    number = float(value)
    if number >= 0.0:
        return float(math.floor(number + 0.5))
    return float(math.ceil(number - 0.5))


def _tty_for_usb(usb_names) -> str | None:
    if isinstance(usb_names, str):
        usb_names = (usb_names,)
    for usb_name in usb_names:
        root = f"/sys/bus/usb/devices/{usb_name}:1.0"
        if not os.path.isdir(root):
            continue
        for dirpath, dirnames, _filenames in os.walk(root):
            for name in dirnames:
                if name.startswith("ttyUSB") and os.path.exists(f"/dev/{name}"):
                    return f"/dev/{name}"
    return None


def find_arm_port(arm_name: str) -> str | None:
    """Prefer the fixed USB port. Fall back to the Force.cpp tty node.

    tty numbers shift on replug, so never fall back onto the other arm's adapter.
    """
    spec = FORCE_ARMS[arm_name]
    by_usb = _tty_for_usb(spec["usb"])
    if by_usb:
        return by_usb
    taken = {_tty_for_usb(other["usb"]) for name, other in FORCE_ARMS.items() if name != arm_name}
    if _exists(spec["port"]) and spec["port"] not in taken:
        return spec["port"]
    return None


def _exists(path: str) -> bool:
    return os.path.exists(path)


def parse_packet(packet: bytes):
    if len(packet) < _PACKET_LEN or packet[0] != 0xAA or packet[1] != 0x55:
        return None
    data = struct.unpack_from("<6f", packet, 6)
    if not _is_valid(data):
        return None
    return data


def _is_valid(data) -> bool:
    for value in data[:3]:
        if value < _FX_MIN or value > _FX_MAX:
            return False
    for value in data[3:]:
        if value < _T_MIN or value > _T_MAX:
            return False
    return True


class _Lowpass:
    def __init__(self, cutoff_hz: float = FORCE_CUTOFF_HZ, sample_hz: float = FORCE_SAMPLE_HZ):
        dt = 1.0 / sample_hz
        rc = 1.0 / (2.0 * math.pi * cutoff_hz)
        self.alpha = dt / (rc + dt)
        self._ready = False
        self._force = [0.0, 0.0, 0.0]
        self._torque = [0.0, 0.0, 0.0]

    def apply(self, data):
        raw_force = data[:3]
        raw_torque = data[3:]
        if not self._ready:
            self._force = [float(v) for v in raw_force]
            self._torque = [float(v) for v in raw_torque]
            self._ready = True
        else:
            alpha = self.alpha
            self._force = [
                alpha * float(raw) + (1.0 - alpha) * prev
                for raw, prev in zip(raw_force, self._force)
            ]
            self._torque = [
                alpha * float(raw) + (1.0 - alpha) * prev
                for raw, prev in zip(raw_torque, self._torque)
            ]
        return tuple(self._force + self._torque)


class WristForceSensors:
    """Read both wrist sensors and hand each sample to on_sample(arm, raw, filtered).

    on_sample runs as soon as the packet is parsed, so time.time() inside it is the sample time.
    """

    def __init__(self):
        self._stop = threading.Event()
        self._threads = []
        self._missing_announced = False

    def start(self, on_sample) -> None:
        if self._threads:
            return
        self._stop.clear()
        for arm_name in FORCE_ARMS:
            thread = threading.Thread(
                target=self._loop,
                args=(arm_name, on_sample),
                name=f"{arm_name}-arm-force",
                daemon=True,
            )
            thread.start()
            self._threads.append(thread)

    def stop(self) -> None:
        self._stop.set()
        for thread in self._threads:
            thread.join(timeout=2.0)
        self._threads = []

    def _loop(self, arm_name: str, on_sample) -> None:
        spec = FORCE_ARMS[arm_name]
        while not self._stop.is_set():
            port = find_arm_port(arm_name)
            if port is None:
                if not self._missing_announced:
                    self._missing_announced = True
                    print(
                        "[Force] wrist sensor port not found "
                        f"(left {' or '.join(LEFT_ARM_USB)} / {LEFT_ARM_FORCE_PORT}, "
                        f"right {RIGHT_ARM_USB} / {RIGHT_ARM_FORCE_PORT}). "
                        "Load the serial driver once per boot: "
                        "sudo modprobe usbserial && "
                        "sudo insmod /home/ubuntu/ch341-5.15.199-tegra.ko"
                    )
                self._stop.wait(2.0)
                continue
            try:
                self._read_port(arm_name, port, spec["topic"], on_sample)
            except Exception as exc:
                print(f"[Force] {arm_name} {port} read failed: {exc}")
                self._stop.wait(1.0)

    def _read_port(self, arm_name: str, port: str, topic: str, on_sample) -> None:
        import serial

        ser = serial.Serial(port, 115200, timeout=0.05)
        filt = _Lowpass()
        try:
            print(f"[Force] {arm_name} arm connected {port} -> {topic}")
            self._configure(ser)
            ser.write(b"AT+GSD\r\n")
            buf = bytearray()
            last_packet = time.monotonic()
            while not self._stop.is_set():
                if time.monotonic() - last_packet > FORCE_STALL_TIMEOUT_S:
                    print(
                        f"[Force] {arm_name} {port}: no data for {FORCE_STALL_TIMEOUT_S:.0f}s, reconnecting"
                    )
                    return
                # A fixed-size read blocks until ~8 packets arrive and bunches samples into
                # 50 ms bursts, which breaks their host timestamps for 30 Hz alignment.
                chunk = ser.read(max(1, ser.in_waiting))
                if not chunk:
                    continue
                buf.extend(chunk)
                while len(buf) >= _PACKET_LEN:
                    idx = 0
                    limit = len(buf) - 1
                    while idx < limit and not (buf[idx] == 0xAA and buf[idx + 1] == 0x55):
                        idx += 1
                    if idx:
                        del buf[:idx]
                    if len(buf) < _PACKET_LEN:
                        break
                    packet = bytes(buf[:_PACKET_LEN])
                    del buf[:_PACKET_LEN]
                    data = parse_packet(packet)
                    if data is None:
                        continue
                    last_packet = time.monotonic()
                    filtered = filt.apply(data)
                    try:
                        on_sample(arm_name, data, filtered)
                    except Exception as exc:
                        print(f"[Force] {arm_name} publish failed: {exc}")
        finally:
            ser.close()
            print(f"[Force] {arm_name} arm disconnected {port}")

    @staticmethod
    def _configure(ser) -> None:
        for cmd in ("AT+SMPF=100", "AT+DCPCU=MVPV", "AT+DCPM=?"):
            ser.write((cmd + "\r\n").encode())
            ser.flush()
            time.sleep(0.1)
            waiting = ser.in_waiting
            if waiting:
                ser.read(waiting)
