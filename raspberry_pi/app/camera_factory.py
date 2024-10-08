"""Choose a camera implementation based on identity."""

# pylint: disable=import-outside-toplevel
# pylint: disable=too-few-public-methods

from app.config.identity import Identity
from app.camera import Camera


class CameraFactory:
    @staticmethod
    def get(identity: Identity) -> list[Camera]:
        try:
            # this will fail if we're not running on a Raspberry Pi.
            from app.real_camera import RealCamera

            num_cameras: int = CameraFactory.get_num_cameras(identity)
            return [RealCamera(identity, x) for x in range(num_cameras)]

        except ImportError:
            from app.fake_camera import FakeCamera

            return [FakeCamera()]

    @staticmethod
    def get_num_cameras(identity: Identity) -> int:
        match identity:
            case (
                Identity.RIGHTAMP
                | Identity.LEFTAMP
                | Identity.SHOOTER
                | Identity.GAME_PIECE
            ):
                return 1
            case Identity.DEV | Identity.FLIPPED | Identity.UNKNOWN:
                return 0
            case _:
                raise ValueError(f"Unknown identity: {identity}")
