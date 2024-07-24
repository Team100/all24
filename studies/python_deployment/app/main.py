""" This is the coprocessor main loop.

It takes data from the camera and the gyro, and publishes it
on network tables.
"""
from app import example1
from app.camera import Camera
from app.tag_detector import TagDetector
from app2 import example3


def main():
    print("main")
    camera = Camera()
    tag_detector = TagDetector()
    try:
        while True:
            request = camera.capture_request()
            try:
                tag_detector.analyze(request)
            finally:
                request.release()
    finally:
        camera.stop()
            

    example1.do_example1()
    example3.do_example3()
