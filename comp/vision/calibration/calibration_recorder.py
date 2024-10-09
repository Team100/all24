"""Calibration Recorder

This just takes frames in the same way as the prod detectors, and writes them to disk.
"""

# pylint: disable=C0115,C0116,E1101,R0903

import os
import pprint
from typing import Any, cast

import cv2
import ntcore
import numpy as np
from cscore import CameraServer
from numpy.typing import NDArray
from picamera2 import CompletedRequest, Picamera2  # type: ignore


class TagFinder:
    IMAGE_DIR = "images"

    def __init__(self, width: int, height: int) -> None:
        self.width = width
        self.height = height

        self.fx = 666
        self.fy = 666

        self.output_stream = CameraServer.putVideo("Processed", width, height)

        # make a place to put example images
        if not os.path.exists(TagFinder.IMAGE_DIR):
            os.mkdir(TagFinder.IMAGE_DIR)
        # to keep track of images to write
        self.img_ts_sec = 0

    def analyze(self, request: CompletedRequest) -> None:
        buffer: NDArray[np.uint8] = request.make_buffer("lores")  # type: ignore

        y_len = self.width * self.height

        # truncate, ignore chrominance. this makes a view, very fast (300 ns)
        img = np.frombuffer(buffer, dtype=np.uint8, count=y_len)

        # this  makes a view, very fast (150 ns)
        img = img.reshape((self.height, self.width))

        # TODO: crop regions that never have targets
        # this also makes a view, very fast (150 ns)
        # img = img[int(self.height / 4) : int(3 * self.height / 4), : self.width]
        # for now use the full frame
        # TODO: probably remove this
        img = img[: self.height, : self.width]

        now_us = ntcore._now()  # pylint:disable=W0212

        self.output_stream.putFrame(img)  # type: ignore

        # Write some of the files for later analysis
        # To retrieve these files, use:
        # scp pi@10.1.0.11:images/* .
        # These will accumulate forever so remember to clean it out:
        # ssh pi@10.1.0.11 "rm images/img*"
        # return

        now_s = now_us // 1000000  # once per second
        if now_s > self.img_ts_sec:
            self.img_ts_sec = now_s
            filename = TagFinder.IMAGE_DIR + "/img" + str(now_s) + ".png"
            cv2.imwrite(filename, img)


def main() -> None:

    camera = Picamera2()

    print("SENSOR MODES AVAILABLE")
    pprint.pprint(camera.sensor_modes)  # type:ignore

    model: str = cast(str, camera.camera_properties["Model"])  # type: ignore
    print("MODEL: " + model)

    if model == "imx708_wide":
        print("V3 Wide Camera")
        # full frame is 4608x2592; this is 2x2
        fullwidth = 2304
        fullheight = 1296
        # medium detection resolution, compromise speed vs range
        width = 1152
        height = 648
    elif model == "imx219":
        print("V2 Camera")
        # full frame, 2x2, to set the detector mode to widest angle possible
        fullwidth = 1664  # slightly larger than the detector, to match stride
        fullheight = 1232
        # medium detection resolution, compromise speed vs range
        width = 832
        height = 616
    elif model == "imx296":
        print("GS Camera")
        # full frame, 2x2, to set the detector mode to widest angle possible
        fullwidth = 1408  # slightly larger than the detector, to match stride
        fullheight = 1088
        # medium detection resolution, compromise speed vs range
        width = 1408
        height = 1088
    else:
        print("UNKNOWN CAMERA: " + model)
        fullwidth = 100
        fullheight = 100
        width = 100
        height = 100

    camera_config: dict[str, Any] = camera.create_still_configuration(  # type:ignore
        # 2 buffers => low latency (32-48 ms), low fps (15-20)
        # 5 buffers => mid latency (40-55 ms), high fps (22-28)
        # 3 buffers => high latency (50-70 ms), mid fps (20-23)
        # robot goes at 50 fps, so roughly a frame every other loop
        # fps doesn't matter much, so minimize latency
        buffer_count=2,
        main={
            "format": "YUV420",
            "size": (fullwidth, fullheight),
        },
        lores={"format": "YUV420", "size": (width, height)},
        controls={
            # these manual controls are useful sometimes but turn them off for now
            # because auto mode seems fine
            # fast shutter means more gain
            # "AnalogueGain": 8.0,
            # try faster shutter to reduce blur.  with 3ms, 3 rad/s seems ok.
            # "ExposureTime": 3000,
            # limit auto: go as fast as possible but no slower than 30fps
            # "FrameDurationLimits": (5000, 33333),  # 41 fps
            # noise reduction takes time, don't need it.
            "NoiseReductionMode": 0,  # libcamera.controls.draft.NoiseReductionModeEnum.Off,
        },
    )

    print("\nREQUESTED CONFIG")
    print(camera_config)  # type:ignore
    camera.align_configuration(camera_config)  # type:ignore
    print("\nALIGNED CONFIG")
    print(camera_config)  # type:ignore
    camera.configure(camera_config)  # type:ignore
    print("\nCONTROLS")
    print(camera.camera_controls)  # type:ignore

    output = TagFinder(width, height)
    camera.start()  # type:ignore
    try:
        while True:
            # the most recent completed frame, from the recent past
            request: CompletedRequest = camera.capture_request()  # type:ignore
            try:
                output.analyze(request)
            finally:
                # the frame is owned by the camera so remember to release it
                request.release()
    finally:
        camera.stop()


main()
