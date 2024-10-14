"""Interface spec for image interpreters."""

from typing import Protocol

from app.camera.camera_protocol import Request

# pylint: disable=R0903


class Interpreter(Protocol):
    def analyze(self, req: Request) -> None: ...
