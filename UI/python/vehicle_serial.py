"""Zybo vehicle heartbeat and FSR status for the Qt HUD.

The standalone car_control_fsr.py protocol is kept here without its Tk window.
Only this object owns the Zybo serial port while the HUD is running.
"""

from dataclasses import dataclass
import re
import time


STATUS_RE = re.compile(
    r"(?:\[ZYBO\]\s*)?SENSOR_SEQ=(?P<sequence>\d+)\s+"
    r"ACC_RAW=(?P<accel_raw>-?\d+)\s+A=(?P<accel_level>\d+)\s+"
    r"BRAKE_RAW=(?P<brake_raw>-?\d+)\s+B=(?P<brake_level>\d+)\s+"
    r"SENSOR=(?P<sensor>OK|LOST)\s+STEER=(?P<steering>-?\d+)\s+"
    r"STEER_SRC=(?P<source>[A-Za-z0-9_]+):(?P<steer_status>OK|STALE)\s+"
    r"ESTOP=(?P<estop>[01])\s+PC=(?P<pc>OK|LOST)"
)


@dataclass(frozen=True)
class VehicleStatus:
    sequence: int
    accel_raw: int
    accel_level: int
    brake_raw: int
    brake_level: int
    sensor_ok: bool
    steering: int
    steering_source: str
    steering_ok: bool
    estop: bool
    pc_ok: bool


def parse_status(line):
    match = STATUS_RE.search(line)
    if match is None:
        return None
    value = match.group
    return VehicleStatus(
        sequence=int(value("sequence")),
        accel_raw=int(value("accel_raw")),
        accel_level=int(value("accel_level")),
        brake_raw=int(value("brake_raw")),
        brake_level=int(value("brake_level")),
        sensor_ok=value("sensor") == "OK",
        steering=int(value("steering")),
        steering_source=value("source"),
        steering_ok=value("steer_status") == "OK",
        estop=value("estop") == "1",
        pc_ok=value("pc") == "OK",
    )


def pressure_percent(status, idle_raw=7000, full_raw=22000):
    """Continuous UI input; motor control keeps Zybo's separate 0-5 levels."""
    if full_raw <= idle_raw:
        raise ValueError("full_raw must exceed idle_raw")
    if not status.sensor_ok or status.estop or status.brake_level or not status.accel_level:
        return 0.0
    return max(0.0, min(100.0, (status.accel_raw - idle_raw) * 100.0 / (full_raw - idle_raw)))


class VehicleSerial:
    def __init__(self, port, baudrate=115200):
        self.port = port
        self.baudrate = baudrate
        self.serial = None
        self.error = None
        self.next_open = 0.0
        self.rx_buffer = bytearray()
        self.held_since = {}
        self.emergency_stop = False
        self.status = None
        self.status_at = None

    def key_press(self, key, now=None):
        if key == "space":
            self.emergency_stop = True
        elif key == "r":
            self.emergency_stop = False
        elif key in ("left", "right"):
            self.held_since.setdefault(key, time.monotonic() if now is None else now)

    def key_release(self, key):
        self.held_since.pop(key, None)

    def steering_command(self, now=None):
        if self.emergency_stop:
            return 0
        now = time.monotonic() if now is None else now
        left = self.held_since.get("left")
        right = self.held_since.get("right")
        if (left is None) == (right is None):
            return 0
        start = left if left is not None else right
        angle = min(90, (1 + int(max(0.0, now - start) / 0.1)) * 10)
        return -angle if left is not None else angle

    def fresh_status(self, now=None):
        now = time.monotonic() if now is None else now
        if self.status_at is not None and now - self.status_at < 0.7:
            return self.status
        return None

    def _disconnect(self, error):
        self.error = str(error)
        if self.serial is not None:
            try:
                self.serial.close()
            except Exception:
                pass
        self.serial = None
        self.rx_buffer.clear()
        self.status = None
        self.status_at = None
        self.next_open = time.monotonic() + 1.0

    def poll(self, now=None):
        now = time.monotonic() if now is None else now
        if self.serial is None:
            if now < self.next_open:
                return
            try:
                import serial
                self.serial = serial.Serial(self.port, self.baudrate, timeout=0, write_timeout=0)
                self.error = None
            except Exception as exc:
                self._disconnect(exc)
                return
        try:
            command = f"K,{self.steering_command(now)},{int(self.emergency_stop)}\n"
            self.serial.write(command.encode("ascii"))
            count = min(self.serial.in_waiting, 4096)
            if count:
                self.rx_buffer.extend(self.serial.read(count))
                if len(self.rx_buffer) > 8192:
                    self.rx_buffer = self.rx_buffer[-4096:]
                while b"\n" in self.rx_buffer:
                    line, _, rest = self.rx_buffer.partition(b"\n")
                    self.rx_buffer = bytearray(rest)
                    parsed = parse_status(line.decode("ascii", errors="ignore"))
                    if parsed is not None:
                        self.status = parsed
                        self.status_at = now
        except Exception as exc:
            self._disconnect(exc)

    def close(self):
        if self.serial is not None:
            for _ in range(10):
                try:
                    self.serial.write(b"K,0,1\n")
                except Exception:
                    break
                time.sleep(0.02)
            try:
                self.serial.close()
            except Exception:
                pass
            self.serial = None
