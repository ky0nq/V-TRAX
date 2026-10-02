import sys
import threading
import types
import unittest
from unittest.mock import patch

from camera_discovery import SERVICE_TYPE, discover_camera_url, stream_url_from_info


class FakeServiceInfo:
    port = 81

    def parsed_addresses(self):
        return ["192.168.137.42"]


class FakeZeroconf:
    def __init__(self, ip_version=None):
        self.closed = False

    def get_service_info(self, service_type, name, timeout):
        return FakeServiceInfo()

    def close(self):
        self.closed = True


class FakeBrowser:
    def __init__(self, zeroconf, service_type, handlers):
        assert service_type == SERVICE_TYPE
        handlers[0](zeroconf, service_type, "VisionCam." + service_type, "added")
        self.cancelled = False

    def cancel(self):
        self.cancelled = True


class CameraDiscoveryTests(unittest.TestCase):
    def test_builds_stream_url_from_service_address(self):
        self.assertEqual(stream_url_from_info(FakeServiceInfo()),
                         "http://192.168.137.42:81/stream")

    def test_browses_advertised_camera(self):
        fake = types.SimpleNamespace(
            IPVersion=types.SimpleNamespace(V4Only=4),
            ServiceStateChange=types.SimpleNamespace(Added="added", Updated="updated"),
            ServiceBrowser=FakeBrowser,
            Zeroconf=FakeZeroconf,
        )
        with patch.dict(sys.modules, {"zeroconf": fake}):
            self.assertEqual(discover_camera_url(timeout=0.1),
                             "http://192.168.137.42:81/stream")

    def test_stopped_search_returns_none(self):
        fake = types.SimpleNamespace(
            IPVersion=types.SimpleNamespace(V4Only=4),
            ServiceStateChange=types.SimpleNamespace(Added="added", Updated="updated"),
            ServiceBrowser=lambda *args, **kwargs: types.SimpleNamespace(cancel=lambda: None),
            Zeroconf=FakeZeroconf,
        )
        stopped = threading.Event()
        stopped.set()
        with patch.dict(sys.modules, {"zeroconf": fake}):
            self.assertIsNone(discover_camera_url(timeout=0.1, stop_event=stopped))


if __name__ == "__main__":
    unittest.main()
