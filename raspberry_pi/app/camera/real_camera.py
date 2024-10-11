""" This is a wrapper for Picamera2.

It handles configuration of each camera according to the Pi identity.

For more on the Picamera2 library, see the manual:

https://datasheets.raspberrypi.com/camera/picamera2-manual.pdf

and the source:

https://github.com/raspberrypi/picamera2/
"""

from contextlib import AbstractContextManager
from enum import Enum, unique
from mmap import mmap
from pprint import pprint
from typing import Any

import numpy as np
from numpy.typing import NDArray
from picamera2 import CompletedRequest, Picamera2  # type: ignore
from picamera2.request import MappedArray, _MappedBuffer  # type: ignore
from typing_extensions import override

from app.camera.camera_protocol import Camera, Request, Size
from app.config.identity import Identity
from app.localization.network import Network
from app.util.timer import Timer

Mat = NDArray[np.uint8]


class RealRequest(Request):
    def __init__(self, req: CompletedRequest):
        self._req = req

    @override
    def release(self) -> None:
        self._req.release()

    @override
    def buffer(self) -> AbstractContextManager[mmap]:
        # see CompletedRequest.make_buffer()
        # which makes an np.array from the _MappedBuffer.
        # for greyscale, you can use np.frombuffer().
        #
        # this is AbstractContextManager[mmap] because the flow is:
        # during picamera2.configure(), the DmaAllocator allocates
        # the requested (buffer_count) number of dma buffers, which
        # are mmap.mmap objects.
        # the camera uses DmaAllocator, whose sync property
        # is the DmaSync constructor
        # the _MappedBuffer constructor invokes this constructor
        # to get a sync object, which is the delegate for the _MappedBuffer
        # __enter__() method, which simply returns the mmap above.
        #
        # the reason to use a MappedBuffer is in order to release
        # the dma buffer when we're done with it, which happens
        # in MappedBuffer.__exit__(), which again delegates to the
        # sync object, which uses the appropriate ioctl flags.
        #
        # since _MappedBuffer implements __enter__ and __exit__, we
        # can duck-type it an AbstractContextManager.
        return _MappedBuffer(self._req, "lores")  # type: ignore

    @override
    def array(self) -> AbstractContextManager[MappedArray]:  # type:ignore
        return MappedArray(self._req, "lores")

    @override
    def metadata(self) -> dict[str, Any]:
        return self._req.get_metadata()  # type: ignore


@unique
class Model(Enum):
    V3_WIDE = "imx708_wide"
    V2 = "imx219"
    GS = "imx296"
    UNKNOWN = "unknown"

    @classmethod
    def _missing_(cls, value: object) -> Any:
        return Identity.UNKNOWN

    @staticmethod
    def get(cam: Picamera2) -> "Model":
        model_str: str = cam.camera_properties["Model"]  # type:ignore
        print(f"Camera model string: {model_str}")
        model: Model = Model(model_str)
        print(f"Camera model: {model.name}")
        return model


class RealCamera(Camera):
    def __init__(
        self,
        identity: Identity,
        camera_num: int,
        network: Network,
    ) -> None:
        path = "vision/" + identity.value + "/" + str(camera_num)
        self._capture_time = network.get_double_sender(path + "/capture_time_ms")
        self.cam: Picamera2 = Picamera2(camera_num)
        model: Model = Model.get(self.cam)
        self.size: Size = RealCamera.__size_from_model(model)
        self.camera_config: dict[str, Any] = RealCamera.__get_config(
            identity, self.cam, self.size
        )
        self.mtx = RealCamera.__mtx_from_model(model)
        self.dist = RealCamera.__dist_from_model(model)
        print("SENSOR MODES AVAILABLE")
        pprint(self.cam.sensor_modes)  # type:ignore
        if identity == Identity.FLIPPED:
            # see libcamera/src/libcamera/transform.cpp
            self.camera_config["transform"] = 3

        print("\nREQUESTED CONFIG")
        print(self.camera_config)
        self.cam.align_configuration(self.camera_config)  # type:ignore
        print("\nALIGNED CONFIG")
        print(self.camera_config)
        self.cam.configure(self.camera_config)  # type:ignore
        print("\nCONTROLS")
        print(self.cam.camera_controls)  # type:ignore
        self.cam.start()  # type:ignore

    @override
    def capture_request(self) -> Request:
        capture_start: int = Timer.time_ns()
        req: CompletedRequest = self.cam.capture_request()  # type:ignore
        capture_end: int = Timer.time_ns()
        # capture time is how long we wait for the camera, it should be close to zero.
        capture_time_ms: int = (capture_end - capture_start) // 1000000
        self._capture_time.send(capture_time_ms, 0)
        return RealRequest(req)

    @override
    def stop(self) -> None:
        self.cam.stop()
        print("Camera stop")

    @override
    def get_size(self) -> Size:
        return self.size

    @override
    def get_intrinsic(self) -> Mat:
        return self.mtx

    @override
    def get_dist(self) -> Mat:
        return self.dist

    @staticmethod
    def __size_from_model(model: Model) -> Size:
        match model:
            case Model.V3_WIDE:
                return Size(fullwidth=2304, fullheight=1296, width=1152, height=648)

            case Model.V2:
                return Size(fullwidth=1664, fullheight=1232, width=832, height=616)

            case Model.GS:
                return Size(fullwidth=1408, fullheight=1088, width=1408, height=1088)

            case _:
                return Size(fullwidth=100, fullheight=100, width=100, height=100)

    @staticmethod
    def __get_config(identity: Identity, cam: Picamera2, size: Size) -> dict[str, Any]:
        camera_config: dict[str, Any] = cam.create_still_configuration(  # type:ignore
            # more buffers seem to make the pipeline a little smoother
            buffer_count=5,
            # hang on to one camera buffer (zero copy) and leave one
            # other for the camera to fill.
            queue=True,
            main={
                "format": "YUV420",
                "size": (size.fullwidth, size.fullheight),
            },
            lores={"format": "YUV420", "size": (size.width, size.height)},
            controls={
                "ExposureTime": RealCamera.__get_exposure_time(identity),
                "AnalogueGain": 8,
                # limit auto: go as fast as possible but no slower than 30fps
                # without a duration limit, we slow down in the dark, which is fine
                # "FrameDurationLimits": (5000, 33333),  # 41 fps
                # noise reduction takes A LOT of time (about 100 ms per frame!), don't need it.
                "NoiseReductionMode": 0,  # libcamera.controls.draft.NoiseReductionModeEnum.Off,
                # "ScalerCrop":(0,0,width/2,height/2),
            },
        )
        return camera_config

    @staticmethod
    def __get_exposure_time(identity: Identity) -> int:
        """exposure time in microseconds"""
        match identity:
            case Identity.GLOBAL_RIGHT | Identity.GLOBAL_LEFT:
                return 300  # from b5879a6, works with GS cameras
            case _:
                return 3000  # the old value, works with v2 cameras

    @staticmethod
    def __mtx_from_model(model: Model) -> Mat:
        """Intrinsic matrix."""
        match model:
            case Model.V3_WIDE:
                return np.array(
                    [
                        [498, 0, 584],
                        [0, 498, 316],
                        [0, 0, 1],
                    ]
                )
            case Model.V2:
                return np.array(
                    [
                        [660, 0, 426],
                        [0, 660, 303],
                        [0, 0, 1],
                    ]
                )
            case Model.GS:
                return np.array(
                    [
                        [1680, 0, 728],
                        [0, 1680, 544],
                        [0, 0, 1],
                    ]
                )
            case _:
                return np.array(
                    [
                        [100, 0, 50],
                        [0, 100, 50],
                        [0, 0, 1],
                    ]
                )

    @staticmethod
    def __dist_from_model(model: Model) -> Mat:
        """Minimal distortion matrix with four elements, [k1, k2, p1, p2]
        see https://docs.opencv.org/4.x/d9/d0c/group__calib3d.html"""
        match model:
            case Model.V3_WIDE:
                return np.array([[0.01, -0.0365, 0, 0]])
            case Model.V2:
                return np.array([[-0.003, 0.04, 0, 0]])
            case Model.GS:
                return np.array([[0, 0, 0, 0]])
            case _:
                return np.array([[0, 0, 0, 0]])
