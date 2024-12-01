# pylint: disable=C0301,E0611,E1101,R0903
import numpy as np

from app.camera.camera_protocol import Camera
from app.camera.interpreter_protocol import Interpreter
from app.config.identity import Identity
from app.dashboard.fake_display import FakeDisplay
from app.dashboard.real_display import RealDisplay
from app.localization.note_detector import NoteDetector
from app.localization.tag_detector import TagDetector
from app.network.network import Network


class InterpreterFactory:
    @staticmethod
    def get(
        identity: Identity, cam: Camera, camera_num: int, network: Network
    ) -> Interpreter:
        size = cam.get_size()
        if identity != Identity.UNKNOWN:
            scale = 0.25
        else:
            scale = 1.0
        match identity:
            case Identity.GLOBAL_GAME_PIECE | Identity.GAME_PIECE:
                display = RealDisplay(
                    int(scale * size.width),
                    int(scale * size.height),
                    "tag" + str(camera_num),
                )

                # GREEN TARGET VALUES
                object_lower = np.array((40, 50, 100))
                object_higher = np.array((70, 255, 255))
                return NoteDetector(
                    identity,
                    cam,
                    camera_num,
                    display,
                    network,
                    object_lower,
                    object_higher,
                )
            case (
                Identity.RIGHTAMP
                | Identity.LEFTAMP
                | Identity.SHOOTER
                | Identity.GLOBAL_LEFT
                | Identity.GLOBAL_RIGHT
                | Identity.DEV
            ):
                display = RealDisplay(
                    int(scale * size.width),
                    int(scale * size.height),
                    "note" + str(camera_num),
                )
                return TagDetector(identity, cam, camera_num, display, network)
            case _:
                display = FakeDisplay()
                return TagDetector(identity, cam, camera_num, display, network)
