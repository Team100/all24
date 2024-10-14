""" This is a wrapper for Picamera2.

It handles configuration of each camera according to the Pi identity.

For more on the Picamera2 library, see the manual:

https://datasheets.raspberrypi.com/camera/picamera2-manual.pdf

and the source:

https://github.com/raspberrypi/picamera2/
"""

from contextlib import AbstractContextManager
from enum import Enum, unique
from pprint import pprint
from typing import Any

import numpy as np
from numpy.typing import NDArray
from picamera2 import CompletedRequest, Picamera2  # type: ignore
from picamera2.request import _MappedBuffer  # type: ignore
from typing_extensions import Buffer, override

from app.camera.camera_protocol import Camera, Request, Size
from app.config.identity import Identity
from app.network.real_network import Network
from app.util.timer import Timer

Mat = NDArray[np.uint8]


class RealRequest(Request):
    def __init__(self, req: CompletedRequest):
        # Before we get a CompletedRequest, its constructor has used the
        # camera allocator sync property to:
        # * instantiate a DMA allocator sync for each buffer
        # * tell the camera allocator to mark the buffers as 'in use'
        # * __enter__() each buffer's DmaSync, which calls ioctl DMA_BUF_SYNC_START
        self._req = req

    @override
    def metadata(self) -> dict[str, Any]:
        return self._req.get_metadata()  # type: ignore

    @override
    def rgb(self) -> AbstractContextManager[Buffer]:
        return self._buffer("main")

    @override
    def yuv(self) -> AbstractContextManager[Buffer]:
        return self._buffer("lores")

    def _buffer(self, stream: str) -> AbstractContextManager[Buffer]:
        # Returns AbstractContextManager[Buffer] because the flow is:
        #
        # During picamera2.configure(), the DmaAllocator allocates
        # the requested (buffer_count) number of dma buffers, which
        # are mmap.mmap objects, which are Buffers.
        #
        # The camera uses DmaAllocator, whose sync property
        # is the DmaSync constructor.
        #
        # The _MappedBuffer constructor invokes the DmaSync constructor
        # to get a DmaSync.  _MappedBuffer.__enter__() delegates to DmaSync to
        # do the ioctl DMA_BUF_SYNC_START (again), and return the mmap.
        #
        # _MappedBuffer.__exit__() also delegates to DmaSync, which implements
        # DMA_BUF_SYNC_END.
        #
        # since _MappedBuffer implements __enter__ and __exit__, we
        # can duck-type it an AbstractContextManager.
        #
        # Note that when the _MappedBuffer is __exit__'ed, the DMA buffer should
        # not be touched anymore, which means not using numpy views on it.
        # The Picamera code addresses this in CompletedRequest.make_buffer by
        # *copying* the buffer, but we definitely don't want to do that.  Just
        # do all your work on the buffer within the scope of the context manager.
        #
        # This use of _MappedBuffer is not necessary for the buffer-reservation
        # (it's done by the CompletedRequest), but using _MappedBuffer is, by far,
        # the easiest way to get at the mmap buffer.
        #
        # To use the buffer, you can pass it to np.frombuffer().
        return _MappedBuffer(self._req, stream)  # type: ignore

    @override
    def release(self) -> None:
        # Calls DmaSync.__exit__() which invokes ioctl DMA_BUF_SYNC_END
        # to release the DMA buffer, and unmark the app-level 'in use' flag.
        # Note that the _MappedBuffer has already done the ioctl work, so this
        # is redundant.
        self._req.release()


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
        self.rolling = RealCamera.__rolling_from_model(model)
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
        # optimal alignment makes the ISP a little faster
        self.cam.align_configuration(self.camera_config, optimal=True)  # type:ignore
        print("\nALIGNED CONFIG")
        print(self.camera_config)
        self.cam.configure(self.camera_config)  # type:ignore
        if (
            self.camera_config["sensor"]["output_size"]
            != self.cam.camera_config["sensor"]["output_size"]  # type:ignore
        ):
            raise ValueError(
                "desired sensor size must match selected sensor size",
                self.camera_config["sensor"]["output_size"],
                self.cam.camera_config["sensor"]["output_size"],  # type:ignore
            )
        print("\nCONTROLS")
        print(self.cam.camera_controls)  # type:ignore
        self.cam.start()  # type:ignore

    @override
    def capture_request(self) -> Request:
        capture_start: int = Timer.time_ns()
        req: CompletedRequest = self.cam.capture_request()  # type:ignore
        capture_end: int = Timer.time_ns()
        # capture time is how long we wait for the camera, it should be close to zero.
        capture_time_ms = (capture_end - capture_start) / 1000000
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

    @override
    def is_rolling_shutter(self) -> bool:
        return self.rolling

    @staticmethod
    def __size_from_model(model: Model) -> Size:
        match model:
            case Model.V3_WIDE:
                return Size(
                    sensor_width=2304, sensor_height=1296, width=1152, height=648
                )

            case Model.V2:
                return Size(
                    sensor_width=1640, sensor_height=1232, width=832, height=616
                )

            case Model.GS:
                return Size(
                    sensor_width=1408, sensor_height=1088, width=1408, height=1088
                )

            case _:
                return Size(sensor_width=100, sensor_height=100, width=100, height=100)

    @staticmethod
    def __get_config(identity: Identity, cam: Picamera2, size: Size) -> dict[str, Any]:
        """Consult https://datasheets.raspberrypi.com/camera/picamera2-manual.pdf"""
        camera_config: dict[str, Any] = cam.create_still_configuration(  # type:ignore
            # more buffers seem to make the pipeline a little smoother
            buffer_count=5,
            # hang on to one camera buffer (zero copy) and leave one
            # other for the camera to fill.
            queue=True,
            # TODO: make this direct sensor configuration actually work
            #
            sensor={
                "output_size": (size.sensor_width, size.sensor_height),
                "bit_depth": 10,
            },
            # TODO: make main RGB so we can provide it to color-desiring interpreters
            main={
                # see Appendix A for format strings.  Note "BGR" really means "RGB" (!)
                "format": "RGB888",
                "size": (size.width, size.height),
            },
            lores={
                "format": "YUV420",
                "size": (size.width, size.height),
            },
            raw=None,
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

    @staticmethod
    def __rolling_from_model(model: Model) -> bool:
        return model != Model.GS
