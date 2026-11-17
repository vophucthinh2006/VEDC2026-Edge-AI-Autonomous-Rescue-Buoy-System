"""No worker thread may name a method like an attribute threading.Thread sets on itself: the attribute
hides the method. Python 3.13 (the Pi 5) added Thread._handle, and Stm32Uart._handle stopped the link."""
import importlib
import inspect
import threading
import unittest

MODULES = ["uart_comm", "helper_board", "ai_vision", "announcer", "lidar_serial", "viewer", "bt_manual"]


class ThreadNamesTest(unittest.TestCase):
    def test_no_method_hidden_by_a_thread_attribute(self):
        reserved = set(vars(threading.Thread())) | {"_handle", "_started", "_ident", "_tstate_lock", "_is_stopped"}
        for name in MODULES:
            try:
                module = importlib.import_module(f"modules.{name}")
            except ImportError:
                continue   # a worker whose libraries are not installed here (cv2, serial)
            for cls_name, cls in inspect.getmembers(module, inspect.isclass):
                if issubclass(cls, threading.Thread) and cls is not threading.Thread and cls.__module__ == module.__name__:
                    clash = reserved & {k for k, v in vars(cls).items() if callable(v)}
                    self.assertFalse(clash, f"{cls_name}: {sorted(clash)}")


if __name__ == "__main__":
    unittest.main()
