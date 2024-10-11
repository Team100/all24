"""Capture a request, analyze it, release it, repeat."""

# pylint: disable=R0903

from threading import Event

from typing_extensions import override

from app.camera.camera_protocol import Camera
from app.camera.interpreter_protocol import Interpreter
from app.framework.looper import Looper


class CameraLoop(Looper):
    def __init__(
        self,
        interpreter: Interpreter,
        camera: Camera,
        done: Event,
    ) -> None:
        self.interpreter = interpreter
        self.camera = camera
        self.done = done

    @override
    def run(self) -> None:
        # the camera loop runs as fast as possible,
        # because the interpreter is slower than the camera.
        try:
            while True:
                if self.done.is_set():  # exit cleanly
                    return
                req = self.camera.capture_request()
                try:
                    self.interpreter.analyze(req)
                finally:
                    req.release()
        finally:
            self.camera.stop()
            self.done.set()
