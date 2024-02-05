import cv2
import libcamera
import numpy as np
import time
from enum import Enum

from cscore import CameraServer
from ntcore import NetworkTableInstance
from picamera2 import Picamera2
from wpimath.geometry import Transform3d
import dataclasses
from wpiutil import wpistruct

@wpistruct.make_wpistruct
@dataclasses.dataclass
class NotePosition:
    x: int
    y: int

class Camera(Enum):
    """Keep this synchronized with java team100.config.Camera."""

    FRONT = "1000000013c9c96c"  # "2"
    REAR = "100000004e0a1fb9"  # "1"
    LEFT = "10000000a7c673d9"  # "4"
    RIGHT = "10000000a7a892c0"  # "3"
    UNKNOWN = None

    @classmethod
    def _missing_(cls, value):
        return Camera.UNKNOWN
    
class GamePieceFinder:

    def __init__(self, serial, topic_name, camera_params):
        self.object_lower = (90 , 90, 200)
        self.serial = serial
        self.object_higher = (120, 255, 255)
        self.width = camera_params[0]
        self.height = camera_params[1]
        self.frame_time = 0
        self.theta = 0
        self.topic_name = topic_name
        self.initialize_nt()    
        self.output_stream = CameraServer.putVideo("Processed", self.width, self.height)

    def initialize_nt(self):
        """Start NetworkTables with Rio as server, set up publisher."""
        self.inst = NetworkTableInstance.getDefault()
        self.inst.startClient4("gamepiece_finder24")
        # this is always the RIO IP address; set a matching static IP on your
        # laptop if you're using this in simulation.
        # self.inst.setServer("10.107.191.21")
        self.inst.setServer("10.1.0.2")
        # Table for vision output information
        topic_name = "vision/" + self.serial
        self.vision_fps = self.inst.getDoubleTopic(topic_name + "/fps").publish()
        self.vision_latency = self.inst.getDoubleTopic(
            topic_name + "/latency"
        ).publish()

        # work around https://github.com/robotpy/mostrobotpy/issues/60
        self.inst.getStructTopic("bugfix", NotePosition).publish().set(
            NotePosition(0,0)
        )

        self.vision_nt_struct = self.inst.getStructArrayTopic(
            topic_name + "/NotePosition24", NotePosition
        ).publish()

    def find_object(self, img):
        range = cv2.inRange(img, self.object_lower, self.object_higher)
        img_rgb = cv2.cvtColor(img, cv2.COLOR_HSV2RGB)
        floodfill = range.copy()
        h, w = range.shape[:2]
        mask = np.zeros((h+2, w+2), np.uint8)
        cv2.floodFill(floodfill, mask, (0,0), 255)
        floodfill_inv = cv2.bitwise_not(floodfill)
        img_floodfill = range | floodfill_inv
        median = cv2.medianBlur(img_floodfill, 5)
        contours, hierarchy = cv2.findContours(
            median, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        objects = []
        for c in contours:
            _, _, cnt_width, cnt_height = cv2.boundingRect(c)
            if (cnt_width/cnt_height < 1):
                continue
            if (cnt_width == 832 and cnt_height == 616):
                continue
            if ():
                (cnt_height < 10 or cnt_width < 10)
                continue
            mmnts = cv2.moments(c)
            if (mmnts["m00"] == 0):
                continue

            cX = int(mmnts["m10"] / mmnts["m00"])
            cY = int(mmnts["m01"] / mmnts["m00"])
            
            # translation_x = (cX-self.width/2)* \
            #     (self.object_height*math.cos(self.theta)/cnt_height)
            # translation_y = (cY-self.height/2) * \
            #     (self.object_height*math.cos(self.theta)/cnt_height)
            # translation_z = (self.object_height*self.scale_factor*math.cos(self.theta))/(cnt_height)
            objects.append(
                NotePosition(cX,cY)
            )
            self.draw_result(img_rgb, c, cX, cY)
        self.output_stream.putFrame(img_rgb)
        return objects

    def draw_result(self, img, cnt, cX, cY):
        # float_formatter = {"float_kind": lambda x: f"{x:4.1f}"}
        cv2.drawContours(img, [cnt], -1, (0, 255, 0), 2)
        cv2.circle(img, (int(cX), int(cY)), 7, (0, 0, 0), -1)
        # cv2.putText(img, f"t: {np.array2string(wpi_t.flatten(), formatter=float_formatter)}", (int(cX) - 20, int(cY) - 20),
        #             cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 2)
        

    def analyze(self, request):
        
        buffer = request.make_array("lores")
        metadata = request.get_metadata()

        # img = np.frombuffer(buffer, dtype=np.uint8)
        img = buffer
        # print(buffer.shape)
        # img = img.reshape((self.height, self.width))
        img_bgr = cv2.cvtColor(img, cv2.COLOR_YUV420p2BGR)
        # img_bgr = img_bgr[201:616,:,:]
        img_hsv = cv2.cvtColor(img_bgr, cv2.COLOR_BGR2HSV)
        img_hsv = np.ascontiguousarray(img_hsv)
        objects = self.find_object(img_hsv)
        current_time = time.time()
        total_et = current_time - self.frame_time
        self.frame_time = current_time
        fps = 1 / total_et
        self.vision_nt_struct.set(objects)
        self.vision_fps.set(fps)
        sensor_timestamp = metadata["SensorTimestamp"]
        # include all the work above in the latency
        system_time_ns = time.clock_gettime_ns(time.CLOCK_BOOTTIME)
        time_delta_ms = (system_time_ns - sensor_timestamp) // 1000000
        self.vision_latency.set(time_delta_ms)
        self.inst.flush()
        
def getserial():
    with open("/proc/cpuinfo", "r", encoding="ascii") as cpuinfo:
        for line in cpuinfo:
            if line[0:6] == "Serial":
                return line[10:26]
    return ""

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
    serial = getserial()
    identity = Camera(serial)
    if identity == Camera.REAR or identity == Camera.FRONT:
        camera_config["transform"] = libcamera.Transform(hflip=1, vflip=1)
    # Roborio IP: 10.1.0.2
    # Pi IP: 10.1.0.21
    camera_params = [width, 616]
    topic_name = "pieces"
    serial = getserial()
    output = GamePieceFinder(serial,topic_name, camera_params)
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
