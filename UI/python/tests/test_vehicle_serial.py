import sys
import types
import unittest
from pathlib import Path
from unittest.mock import patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from vehicle_serial import VehicleSerial, parse_status, pressure_percent


REPORT = (
    "SENSOR_SEQ=228 ACC_RAW=14500 A=3 BRAKE_RAW=2906 B=0 "
    "SENSOR=OK STEER=-30 STEER_SRC=PC:OK ESTOP=0 PC=OK\r\n"
)


class FakePort:
    def __init__(self):
        self.incoming = bytearray(REPORT.encode("ascii"))
        self.writes = []
        self.closed = False

    @property
    def in_waiting(self):
        return len(self.incoming)

    def read(self, count):
        data = bytes(self.incoming[:count])
        del self.incoming[:count]
        return data

    def write(self, data):
        self.writes.append(data)
        return len(data)

    def close(self):
        self.closed = True


class VehicleSerialTests(unittest.TestCase):
    def test_status_and_continuous_pressure(self):
        status = parse_status("[ZYBO] " + REPORT)
        self.assertEqual((status.accel_raw, status.accel_level, status.steering), (14500, 3, -30))
        self.assertEqual(pressure_percent(status), 50.0)
        self.assertIsNone(parse_status("unrelated Vitis message"))
        self.assertEqual(pressure_percent(status.__class__(**{**vars(status), "brake_level": 1})), 0)
        self.assertEqual(pressure_percent(status.__class__(**{**vars(status), "sensor_ok": False})), 0)

    def test_heartbeat_keys_stale_status_and_safe_close(self):
        fake = FakePort()
        module = types.SimpleNamespace(Serial=lambda *args, **kwargs: fake)
        with patch.dict(sys.modules, {"serial": module}), patch("vehicle_serial.time.sleep"):
            link = VehicleSerial("COM5")
            link.key_press("left", now=100.0)
            self.assertEqual(link.steering_command(now=100.0), -10)
            self.assertEqual(link.steering_command(now=100.31), -40)
            link.poll(now=100.31)
            self.assertEqual(fake.writes[0], b"K,-40,0\n")
            self.assertEqual(link.fresh_status(now=100.5).steering, -30)
            self.assertIsNone(link.fresh_status(now=101.1))
            link.key_press("space")
            link.poll(now=100.6)
            self.assertEqual(fake.writes[1], b"K,0,1\n")
            link.close()
            self.assertTrue(fake.closed)
            self.assertEqual(fake.writes[-10:], [b"K,0,1\n"] * 10)


if __name__ == "__main__":
    unittest.main()
