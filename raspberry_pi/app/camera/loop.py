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
        interpreters: list[Interpreter],
        camera: Camera,
        done: Event,
    ) -> None:
        super().__init__(done)
        self.interpreters = interpreters
        self.camera = camera

    @override
    def execute(self) -> None:
        req = self.camera.capture_request()
        try:
            for i in self.interpreters:
                i.analyze(req)
        finally:
            req.release()

    @override
    def end(self) -> None:
        self.camera.stop()
