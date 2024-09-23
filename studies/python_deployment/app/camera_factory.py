"""Choose a camera implementation based on identity."""

# pylint: disable=import-outside-toplevel
# pylint: disable=too-few-public-methods

from app.identity import Identity
from app.camera import Camera

class CameraFactory:
    @staticmethod
    def get(identity: Identity) -> Camera:
        try:
            # this will fail if we're not running on a Raspberry Pi.
            from app.real_camera import RealCamera
            cameras = []
            for x in range(identity.getNumCameras(identity)):
                if (identity.getNumCameras(identity) == 1):
                    cameras.append(RealCamera(identity, None))
                    continue
                cameras.append(RealCamera(identity, x))
            return cameras

        except ImportError:
            from app.fake_camera import FakeCamera

            return FakeCamera()
