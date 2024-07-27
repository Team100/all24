""" This is a wrapper for Picamera2.

It handles configuration of each camera according to the Pi identity.

For more on the Picamera2 library, see the manual:

https://datasheets.raspberrypi.com/camera/picamera2-manual.pdf

and the source:

https://github.com/raspberrypi/picamera2/
"""

from picamera2 import Picamera2, CompletedRequest # type: ignore


class Request:
    def __init__(self, req: CompletedRequest):
        self.req = req

    def release(self):
        self.req.release()


class Camera:
    def __init__(self):
        self.cam = Picamera2()

    def capture_request(self) -> Request:
        return Request(self.cam.capture_request)

    def start(self):
        self.cam.start()

    def stop(self):
        print("Camera stop")
