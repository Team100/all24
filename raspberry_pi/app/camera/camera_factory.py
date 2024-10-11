""" Choose a camera implementation based on identity.
"""

# pylint: disable=C0415

from app.camera.camera_protocol import Camera
from app.camera.fake_camera import NoteCamera
from app.config.identity import Identity
from app.localization.network import Network


class CameraFactory:
    @staticmethod
    def get(identity: Identity, camera_num: int, network: Network) -> Camera:
        try:
            # this will fail if we're not running on a Raspberry Pi.
            from app.camera.real_camera import RealCamera

            return RealCamera(identity, camera_num, network)

        except ImportError:
            from app.camera.fake_camera import FakeCamera

            if camera_num == 0:
                return FakeCamera()
            return NoteCamera()

    @staticmethod
    def get_num_cameras(identity: Identity) -> int:
        match identity:
            case Identity.UNKNOWN:  # will use FakeCamera and NoteCamera
                return 2
            case (
                Identity.RIGHTAMP
                | Identity.LEFTAMP
                | Identity.SHOOTER
                | Identity.GAME_PIECE
                | Identity.UNKNOWN  # will use FakeCamera
            ):
                return 1
            case Identity.DEV | Identity.FLIPPED:
                return 0
            case _:
                raise ValueError(f"Unknown identity: {identity}")
