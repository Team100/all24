import cv2
import os
import libcamera
import numpy as np
#from tflite_support.task import core
#from tflite_support.task import processor
#from tflite_support.task import vision
#import utils
from cscore import CameraServer
from picamera2 import Picamera2

class GamePieceFinder:

    def __init__(self, topic_name, camera_params):
        self.purple_lower = (150, 100, 0)
        self.purple_higher = (190, 255, 255)
        self.yellow_lower = (15, 100, 0)
        self.yellow_higher = (100, 255, 255)
        self.scale_factor = 1
        self.width = camera_params[0]
        self.height = camera_params[1]
        self.cube_height = .105
        self.cone_height = .105
        self.theta = 0
        self.topic_name = topic_name
        self.output_stream = CameraServer.putVideo("Processed", self.width, self.height)
    
    def analyze(self, request):
        buffer = request.make_array("lores")
        img_bgr = cv2.cvtColor(buffer, cv2.COLOR_YUV420p2BGR)
        img_bgr = img_bgr[201:401,:,:]
        img_hsv = cv2.cvtColor(img_bgr, cv2.COLOR_BGR2HSV)
        img_hsv = np.ascontiguousarray(img_hsv)
        self.img_rgb = cv2.cvtColor(img_hsv, cv2.COLOR_HSV2RGB)
        #imTensor = vision.TensorImage.create_from_array(self.img_rgb)
        #detections=self.detector.detect(imTensor)
        #image=utils.visualize(self.img_rgb, detections)
        self.output_stream.putFrame(self.img_rgb)


def main():
    print("main")
    fullwidth = 1664
    fullheight = 1232
    width = 832
    height = 616
    # option 3: tiny, trade speed for detection distance; two circles, three squraes, ~40ms
    # width=448
    # height=308
    # fast path
    # width=640
    # height=480
    # medium crop
    # width=1920
    # height=1080

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
        "NoiseReductionMode": libcamera.controls.draft.NoiseReductionModeEnum.Off,
        "AwbEnable": False,
        # "AeEnable": False,
        # "AnalogueGain": 1.0
    },
)
    print("REQUESTED")
    print(camera_config)
    camera.align_configuration(camera_config)
    print("ALIGNED")
    print(camera_config)
    camera.configure(camera_config)
    print(camera.camera_controls)

    # Roborio IP: 10.1.0.2
    # Pi IP: 10.1.0.21
    #path = os.getcwd
    #self.model = os.join(path, 'tflite_model','last_float32.tflite')
    camera_params = [width, 400]
    topic_name = "pieces"
    output = GamePieceFinder(topic_name, camera_params)
    #base_options=core.BaseOptions(file_name=model,use_coral=True,num_threads=num_threads)
    #detection_options=processor.DetectionOptions(max_results=8, score_threshold=.3)
    #options=vision.ObjectDetectionOptions(base_options=base_options, detection_options=detection_options)
    #self.detector=vision.ObjectDetector.create_from_options(options)
    camera.start()
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