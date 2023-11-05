from enum import Enum
import cv2
import libcamera
import msgpack
import numpy as np

from cscore import CameraServer
from ntcore import NetworkTableInstance
from picamera2 import Picamera2
from pupil_apriltags import Detector
import math
    
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
        self.vision_nt = inst.getTable("Vision")
        self.vision_nt_msgpack = self.vision_nt.getRawTopic(self.topic_name).publish(
            "msgpack"
        )

    def find_cubes(self, img):
        # 15, 100, 0
        # 40, 255, 255
        #120, 100, 0
        # 145, 255, 255
        purple_range = cv2.inRange(img, self.purple_lower, self.purple_higher)
        img_rgb = cv2.cvtColor(img, cv2.COLOR_HSV2RGB)
        floodfill = purple_range.copy()
        h, w = purple_range.shape[:2]
        mask = np.zeros((h+2, w+2), np.uint8)
        cv2.floodFill(floodfill, mask, (0,0), 255)
        floodfill_inv = cv2.bitwise_not(floodfill)
        img_floodfill = purple_range | floodfill_inv
        median = cv2.medianBlur(purple_range, 5)
        contours, hierarchy = cv2.findContours(
            median, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        cubes = {}
        cubes["cubes"] = []
        for c in contours:
            _, _, cnt_width, cnt_height = cv2.boundingRect(c)
            if (cnt_height < 50):
                continue
            if (cnt_height/cnt_width < .5):
                continue
            if (cnt_height/cnt_width > 2):
                continue
            mmnts = cv2.moments(c)
            if (mmnts["m00"] == 0):
                continue

            cX = int(mmnts["m10"] / mmnts["m00"])
            cY = int(mmnts["m01"] / mmnts["m00"])

            translation_x = (cX-self.width/2)* \
                (self.cube_height*math.cos(self.theta)/cnt_height)
            translation_y = (cY-self.height/2) * \
                (self.cube_height*math.cos(self.theta)/cnt_height)
            translation_z = (self.cube_height*self.scale_factor*math.cos(self.theta))/(cnt_height)

            cube = [translation_x, translation_y, translation_z]

            cubes["cubes"].append(
                {
                    "pose_t": cube
                }
            )
        #     self.draw_result(img_rgb, c, cX, cY, cube)
        # self.output_stream.putFrame(img_rgb)
        return cubes
    
    def find_cones(self, img):
        yellow_range = cv2.inRange(img, self.yellow_lower, self.yellow_higher)
        img_rgb = cv2.cvtColor(img, cv2.COLOR_HSV2RGB)
        floodfill = yellow_range.copy()
        h, w = yellow_range.shape[:2]
        mask = np.zeros((h+2, w+2), np.uint8)
        cv2.floodFill(floodfill, mask, (0,0), 255)
        floodfill_inv = cv2.bitwise_not(floodfill)
        img_floodfill = yellow_range | floodfill_inv
        median = cv2.medianBlur(img_floodfill, 5)
        contours, hierarchy = cv2.findContours(
            median, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        cones = {}
        cones["cones"] = []
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
                (self.cone_height*math.cos(self.theta)/cnt_height)
            translation_y = (cY-self.height/2) * \
                (self.cone_height*math.cos(self.theta)/cnt_height)
            translation_z = (self.cone_height*self.scale_factor*math.cos(self.theta))/(cnt_height)

            cone = [translation_x, translation_y, translation_z]

            cones["cones"].append(
                {
                    "pose_t": cone
                }
            )
            self.draw_result(img_rgb, c, cX, cY, cone)
        self.output_stream.putFrame(img_rgb)
        return cones

    def draw_result(self, img, cnt, cX, cY, piece):
        float_formatter = {"float_kind": lambda x: f"{x:4.1f}"}
        wpi_t = np.array([piece[2], -piece[0], -piece[1]])
        cv2.drawContours(img, [cnt], -1, (0, 255, 0), 2)
        cv2.circle(img, (int(cX), int(cY)), 7, (0, 0, 0), -1)
        cv2.putText(img, f"t: {np.array2string(wpi_t.flatten(), formatter=float_formatter)}", (int(cX) - 20, int(cY) - 20),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 2)

    def analyze(self, request):
        buffer = request.make_array("lores")
        # img = np.frombuffer(buffer, dtype=np.uint8)
        img = buffer
        # print(buffer.shape)
        # img = img.reshape((self.height, self.width))
        img_bgr = cv2.cvtColor(img, cv2.COLOR_YUV420p2BGR)
        img_bgr = img_bgr[201:401,:,:]
        img_hsv = cv2.cvtColor(img_bgr, cv2.COLOR_BGR2HSV)
        img_hsv = np.ascontiguousarray(img_hsv)
        cubes = self.find_cubes(img_hsv)
        cones = self.find_cones(img_hsv)
        pieces = {}
        pieces["pieces"] = []
        pieces["pieces"].append(
                {
                    "cubes": cubes
                }
            )
        pieces["pieces"].append(
                {
                    "cones": cones
                }
            )
        posebytes = msgpack.packb(pieces)


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
    camera_params = [width, 200]
    topic_name = "pieces"
    output = GamePieceFinder(topic_name, camera_params)

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
