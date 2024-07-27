""" This is a wrapper for Picamera2.

It handles configuration of each camera according to the Pi identity.

For more on the Picamera2 library, see the manual:

https://datasheets.raspberrypi.com/camera/picamera2-manual.pdf

and the source:

https://github.com/raspberrypi/picamera2/
"""

from contextlib import AbstractContextManager
from dataclasses import dataclass
import mmap
from typing import Any

from picamera2 import Picamera2, CompletedRequest  # type: ignore
from picamera2.request import _MappedBuffer  # type: ignore


class Request:
    def __init__(self, req: CompletedRequest):
        self._req = req

    def release(self) -> None:
        self._req.release()

    def buffer(self) -> AbstractContextManager[mmap.mmap]:
        return _MappedBuffer(self._req, "lores")  # type: ignore

    def metadata(self) -> dict[str, Any]:
        return self._req.get_metadata()  # type: ignore


@dataclass(frozen=True, kw_only=True)
class Size:
    fullwidth: int
    fullheight: int
    width: int
    height: int


class Camera:
    def __init__(self) -> None:
        self.cam: Picamera2 = Picamera2()
        model: str = self.cam.camera_properties["Model"]
        print("\nMODEL " + model)
        self.size: Size = Camera.size_from_model(model)

    def capture_request(self) -> Request:
        return Request(self.cam.capture_request)

    def start(self) -> None:
        self.cam.start()

    def stop(self) -> None:
        print("Camera stop")

    def get_size(self) -> Size:
        return self.size

    @staticmethod
    def size_from_model(model: str) -> Size:
        if model == "imx708_wide":
            print("V3 Wide Camera")
            # full frame is 4608x2592; this is 2x2
            # medium detection resolution, compromise speed vs range
            return Size(fullwidth=2304, fullheight=1296, width=1152, height=648)

        if model == "imx219":
            print("V2 Camera")
            # full frame, 2x2, to set the detector mode to widest angle possible
            # width is slightly larger than the detector, to match stride
            # medium detection resolution, compromise speed vs range
            return Size(fullwidth=1664, fullheight=1232, width=832, height=616)

        if model == "imx296":
            print("GS Camera")
            # full frame, 2x2, to set the detector mode to widest angle possible
            # width is slightly larger than the detector, to match stride
            # medium detection resolution, compromise speed vs range
            return Size(fullwidth=1472, fullheight=1088, width=1472, height=1088)

        print("UNKNOWN CAMERA: " + model)
        return Size(fullwidth=100, fullheight=100, width=100, height=100)

    @staticmethod
    def get_config(cam: Picamera2, size: Size) -> dict[str, Any]:
        camera_config: dict[str, Any] = cam.create_still_configuration(
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
                # these manual controls are useful sometimes but turn them off for now
                # because auto mode seems fine
                # fast shutter means more gain
                # "AnalogueGain": 8.0,
                # try faster shutter to reduce blur.  with 3ms, 3 rad/s seems ok.
                # 3/23/24, reduced to 2ms, even less blur.
                "ExposureTime": 3000,
                "AnalogueGain": 8,
                # limit auto: go as fast as possible but no slower than 30fps
                # without a duration limit, we slow down in the dark, which is fine
                # "FrameDurationLimits": (5000, 33333),  # 41 fps
                # noise reduction takes A LOT of time (about 100 ms per frame!), don't need it.
                "NoiseReductionMode": 0, # libcamera.controls.draft.NoiseReductionModeEnum.Off,
                # "ScalerCrop":(0,0,width/2,height/2),
            },
        )
        return camera_config
