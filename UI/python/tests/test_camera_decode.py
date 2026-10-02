"""Keep Qt's JPEG format argument compatible with PySide6 bindings."""

import ast
from pathlib import Path
import threading
import unittest


class FakeImage:
    called_with = None
    fail = False

    @classmethod
    def fromData(cls, data, format_name):
        cls.called_with = (data, format_name)
        if cls.fail:
            raise ValueError("damaged frame")
        return cls()

    def isNull(self):
        return False


class FakeProviderBase:
    Image = 1

    def __init__(self, kind):
        pass


def load_provider_class():
    source = Path(__file__).resolve().parents[1] / "video_receiver.py"
    tree = ast.parse(source.read_text(encoding="utf-8"))
    klass = next(node for node in tree.body if isinstance(node, ast.ClassDef)
                 and node.name == "CameraProvider")
    namespace = {"QQuickImageProvider": FakeProviderBase, "QImage": FakeImage,
                 "threading": threading}
    exec(compile(ast.Module(body=[klass], type_ignores=[]), str(source), "exec"), namespace)
    return namespace["CameraProvider"]


class CameraDecodeTests(unittest.TestCase):
    def setUp(self):
        self.provider = object.__new__(load_provider_class())
        self.provider.lock = threading.Lock()
        self.provider.image = "previous frame"
        FakeImage.fail = False

    def test_uses_string_format_for_pyside(self):
        frame = b"jpeg data"
        self.assertTrue(self.provider.set_jpeg(frame))
        self.assertEqual(FakeImage.called_with, (frame, "JPEG"))
        self.assertIsInstance(self.provider.image, FakeImage)

    def test_invalid_frame_preserves_previous_image(self):
        FakeImage.fail = True
        self.assertFalse(self.provider.set_jpeg(b"damaged"))
        self.assertEqual(self.provider.image, "previous frame")


if __name__ == "__main__":
    unittest.main()
