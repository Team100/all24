""" This is the coprocessor main loop.

It takes data from the camera and the gyro, and publishes it
on network tables.

You can't run this from the command line.  To run the app,
use the script called "runapp.py" in the raspberry_pi directory
(one level above this one).
"""

# pylint: disable=R0914

from app.camera.camera_factory import CameraFactory
from app.config.identity import Identity
from app.dashboard.real_display import RealDisplay
from app.localization.network import Network
from app.localization.tag_detector import TagDetector
from app.sensors.gyro_factory import GyroFactory


def gyro_only(identity: Identity) -> None:
    print("gyro only")

    gyronetwork = Network(identity, 2)
    gyro = GyroFactory.get(identity, gyronetwork)

    while True:
        gyro.sample()


def single_camera(identity: Identity) -> None:
    print("single camera")

    gyronetwork = Network(identity, 2)
    gyro = GyroFactory.get(identity, gyronetwork)

    network0 = Network(identity, 0)
    camera0 = CameraFactory.get(identity, 0, network0)
    size0 = camera0.get_size()
    display0 = RealDisplay(size0.width, size0.height, 0)
    tag_detector0 = TagDetector(identity, camera0, 0, display0, network0)
    camera0.start()

    try:
        while True:
            req0 = camera0.capture_request()
            try:
                tag_detector0.analyze(req0)
                gyro.sample()
            finally:
                req0.release()
    finally:
        camera0.stop()


def dual_camera(identity: Identity) -> None:
    print("dual cameras")

    gyronetwork = Network(identity, 2)
    gyro = GyroFactory.get(identity, gyronetwork)

    network0 = Network(identity, 0)
    camera0 = CameraFactory.get(identity, 0, network0)
    size0 = camera0.get_size()
    display0 = RealDisplay(size0.width, size0.height, 0)
    tag_detector0 = TagDetector(identity, camera0, 0, display0, network0)
    camera0.start()

    network1 = Network(identity, 1)
    camera1 = CameraFactory.get(identity, 1, network1)
    size1 = camera1.get_size()
    display1 = RealDisplay(size1.width, size1.height, 1)
    tag_detector1 = TagDetector(identity, camera1, 1, display1, network1)
    camera1.start()

    try:
        while True:
            req0 = camera0.capture_request()
            req1 = camera1.capture_request()
            try:
                tag_detector0.analyze(req0)
                tag_detector1.analyze(req1)
                gyro.sample()
            finally:
                req0.release()
                req1.release()
    finally:
        camera0.stop()
        camera1.stop()


def main() -> None:
    print("main")
    identity: Identity = Identity.get()
    num_cameras: int = CameraFactory.get_num_cameras(identity)
    if num_cameras == 0:
        gyro_only(identity)
    if num_cameras == 1:
        single_camera(identity)
    if num_cameras == 2:
        dual_camera(identity)
