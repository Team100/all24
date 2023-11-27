import cv2
import numpy as np
import time
from tflite_support.task import core
from tflite_support.task import processor
from tflite_support.task import vision
import utils
from cscore import CameraServer
from picamera2 import Picamera2

class GamePieceFinder:

    def __init__(self, camera_params):
        self.counter, fps = 0, 0
        self.start_time = time.time()
        self.row_size = 20  # pixels
        self.left_margin = 24  # pixels
        self.text_color = (0, 0, 255)  # red
        self.font_size = 1
        self.font_thickness = 1
        self.fps_avg_frame_count = 10
        model = ('/home/pi/test/examples/lite/examples/object_detection/raspberry_pi/efficientdet_lite0_edgetpu.tflite')
        base_options=core.BaseOptions(file_name=model,use_coral=True,num_threads=4)
        detection_options=processor.DetectionOptions(max_results=8, score_threshold=.3)
        options=vision.ObjectDetectorOptions(base_options=base_options, detection_options=detection_options)
        self.detector=vision.ObjectDetector.create_from_options(options)
        self.width = camera_params[0]
        self.height = camera_params[1]
        self.output_stream = CameraServer.putVideo("Processed", self.width, self.height)
    
    def analyze(self, request):
        buffer = request.make_array("lores")
        img_bgr = cv2.cvtColor(buffer, cv2.COLOR_YUV420p2BGR)
        img_bgr = img_bgr[201:401,:,:]
        img_hsv = cv2.cvtColor(img_bgr, cv2.COLOR_BGR2HSV)
        img_hsv = np.ascontiguousarray(img_hsv)
        self.img_rgb = cv2.cvtColor(img_hsv, cv2.COLOR_HSV2RGB)
        imTensor = vision.TensorImage.create_from_array(self.img_rgb)
        detections=self.detector.detect(imTensor)
        image=utils.visualize(self.img_rgb, detections)
        self.output_stream.putFrame(image)
        if self.counter % self.fps_avg_frame_count == 0:
            end_time = time.time()
            fps = self.fps_avg_frame_count / (end_time - self.start_time)
            self.start_time = time.time()

        # Show the FPS
        fps_text = 'FPS = {:.1f}'.format(fps)
        text_location = (self.left_margin, self.row_size)
        #self.output_stream.putText(image, fps_text, text_location, cv2.FONT_HERSHEY_PLAIN,
                #self.font_size, self.text_color, self.font_thickness)


def main():
    print("main")
    fullwidth = 1664
    fullheight = 1232
    width = 832
    height = 616
  # Start capturing video input from the camera
    camera = Picamera2()
    camera_config = camera.create_still_configuration(
    # one buffer to write, one to read, one in between so we don't have to wait
    buffer_count=6,
    main={
        "format": "YUV420",
        "size": (fullwidth, fullheight),
    },
    lores={"format": "YUV420", "size": (width, height)},
        controls={
        "FrameDurationLimits": (5000, 33333),  # 41 fps
        # noise reduction takes time
        "AwbEnable": False,
        # "AeEnable": False,
        # "AnalogueGain": 1.0
    },
)
    camera.align_configuration(camera_config)
    camera.configure(camera_config)
    camera.start()

    # Roborio IP: 10.1.0.2
    # Pi IP: 10.1.0.21
    camera_params = [width, 200]
    output = GamePieceFinder(camera_params)
    try:
        while True:
            request = camera.capture_request()
            try:
                output.analyze(request)
            finally:
                request.release()
    finally:
        camera.stop()
main()