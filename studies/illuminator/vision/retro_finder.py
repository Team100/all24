from enum import Enum
import cv2
import libcamera
import msgpack
import numpy as np

from cscore import CameraServer
from ntcore import NetworkTableInstance
from picamera2 import Picamera2
import math
    
class RetroFinder:

    def __init__(self, topic_name, camera_params, hsv_lower, hsv_higher):
        self.hsv_lower = hsv_lower
        self.hsv_higher = hsv_higher
        self.scale_factor = 641
        self.width = camera_params[0]
        self.height = camera_params[1]
        self.tape_height = .105
        self.theta = 0
        self.topic_name = topic_name
        self.initialize_nt()
        self.output_stream = CameraServer.putVideo("Processed", self.width, self.height)

    def initialize_nt(self):
        """Start NetworkTables with Rio as server, set up publisher."""
        inst = NetworkTableInstance.getDefault()
        inst.startClient4("retro-finder")
        # this is always the RIO IP address; set a matching static IP on your
        # laptop if you're using this in simulation.
        inst.setServer("10.1.0.2")
        # Table for vision output information
        self.vision_nt = inst.getTable("RetroVision")
        self.vision_nt_msgpack = self.vision_nt.getRawTopic(self.topic_name).publish(
            "msgpack"
        )

    def find(self, img):
        green_range = cv2.inRange(img, self.hsv_lower, self.hsv_higher)
        img_rgb = cv2.cvtColor(img, cv2.COLOR_HSV2RGB)
        floodfill = green_range.copy()
        h, w = green_range.shape[:2]
        mask = np.zeros((h+2, w+2), np.uint8)
        cv2.floodFill(floodfill, mask, (0,0), 255)
        floodfill_inv = cv2.bitwise_not(floodfill)
        img_floodfill = green_range | floodfill_inv
        median = cv2.medianBlur(img_floodfill, 5)
 
        contours, hierarchy = cv2.findContours(
            median, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        tapes = {}
        tapes["tapes"] = []
        for c in contours:
            _, _, cnt_width, cnt_height = cv2.boundingRect(c)
            if (cnt_height < 50):
                continue
            if (cnt_height/cnt_width < 2):
                continue
            if (cnt_height/cnt_width > 5):
                continue
            mmnts = cv2.moments(c)
            
            if (mmnts["m00"] == 0):
                continue
            cX = int(mmnts["m10"] / mmnts["m00"])
            cY = int(mmnts["m01"] / mmnts["m00"])

            translation_x = (cX-self.width/2)* \
                (self.tape_height*math.cos(self.theta)/cnt_height)
            translation_y = (cY-self.height/2) * \
                (self.tape_height*math.cos(self.theta)/cnt_height)
            translation_z = (self.tape_height*self.scale_factor*math.cos(self.theta))/(cnt_height)

            tape = [translation_x, translation_y, translation_z]
            wpi_t = [tape[2], -tape[0], -tape[1]]
            tapes["tapes"].append(
                {
                    "pose_t": wpi_t
                }
            )
            self.draw_result(img_rgb, c, cX, cY, wpi_t)
        self.output_stream.putFrame(img_rgb)
        return tapes

    def draw_result(self, img, cnt, cX, cY, wpi_t):
        wpi_tb = np.array(wpi_t)
        float_formatter = {"float_kind": lambda x: f"{x:4.1f}"}
        cv2.drawContours(img, [cnt], -1, (0, 255, 0), 2)
        cv2.circle(img, (int(cX), int(cY)), 7, (0, 0, 0), -1)
        cv2.putText(img, f"t: {np.array2string(wpi_tb.flatten(), formatter=float_formatter)}", (int(cX) - 20, int(cY) - 20),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)

    def analyze(self, request):
        buffer = request.make_array("lores")
        # img = np.frombuffer(buffer, dtype=np.uint8)
        img = buffer
        # print(buffer.shape)
        # img = img.reshape((self.height, self.width))
        img_bgr = cv2.cvtColor(img, cv2.COLOR_YUV420p2BGR)
        img_bgr = img_bgr[176:426,:,:]
        img_hsv = cv2.cvtColor(img_bgr, cv2.COLOR_BGR2HSV)
        img_hsv = np.ascontiguousarray(img_hsv)
        tapes = self.find(img_hsv)
        posebytes = msgpack.packb(tapes)

        self.vision_nt_msgpack.set(posebytes)

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
        "AeEnable": False,
        #"AeEnable": True, # for testing
        #"AnalogueGain": 4.0
    },
)
    camera_config["transform"] = libcamera.Transform(hflip=1, vflip=1)
    print("REQUESTED")
    print(camera_config)
    camera.align_configuration(camera_config)
    print("ALIGNED")
    print(camera_config)
    camera.configure(camera_config)
    print(camera.camera_controls)

    # Roborio IP: 10.1.0.2
    # Pi IP: 10.1.0.11
    camera_params = [width, 250]
    topic_name = "tapes"
    output = RetroFinder(topic_name, camera_params, (20, 250, 100), (60, 255, 255))

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
