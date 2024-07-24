""" This is a wrapper for Picamera2.

It handles configuration of each camera according to the Pi identity.

For more on the Picamera2 library, see the manual:

https://datasheets.raspberrypi.com/camera/picamera2-manual.pdf

and the source:

https://github.com/raspberrypi/picamera2/
"""


class Camera:
    def __init__(self):
        pass

    def capture_request(self):
        return 0

    def stop(self):
        pass
