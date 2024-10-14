""" Choose a camera implementation based on identity."""

# pylint: disable=C0415

from app.camera.camera_protocol import Camera
from app.config.identity import Identity


class CameraFactory:
    @staticmethod
    def get(identity: Identity, camera_num: int) -> Camera:
        try:
            # this will fail if we're not running on a Raspberry Pi.
            from app.camera.real_camera import RealCamera

            return RealCamera(identity, camera_num)

        except ImportError:
            from app.camera.fake_camera import FakeCamera

            if camera_num == 0:
                return FakeCamera("tag_and_board.jpg", (1100, 620))
            return FakeCamera("blob.jpg")

    @staticmethod
    def get_num_cameras(identity: Identity) -> int:
        match identity:
            case Identity.UNKNOWN:
                return 2
            case (
                Identity.RIGHTAMP
                | Identity.LEFTAMP
                | Identity.SHOOTER
                | Identity.GAME_PIECE
                | Identity.GLOBAL_GAME_PIECE
                | Identity.GLOBAL_LEFT
                | Identity.GLOBAL_RIGHT
                | Identity.DEV # has one v2 camera at the moment
            ):
                return 1
            case Identity.FLIPPED:
                return 0
            case _:
                raise ValueError(f"Unknown identity: {identity}")
