import dataclasses

import ntcore
from wpiutil import wpistruct
from wpimath.geometry import Pose2d,Rotation2d,Transform3d,Rotation3d
import time

@wpistruct.make_wpistruct
@dataclasses.dataclass
class Blip24:
    id: int
    pose: Transform3d

def main():
    inst = ntcore.NetworkTableInstance.getDefault()
    inst.startClient4("struct_demo")
#    inst.setServer("10.1.0.2")
    inst.setServer("192.168.3.18") # my desktop


#    vision_nt_struct = inst.getStructArrayTopic("vision/foo", Pose2d).publish()
    # the array by itself seems not to publish the schema.
#    vision_nt_struct2 = inst.getStructTopic("vision/foo2", Pose2d).publish()
    inst.getStructTopic("bugfix", Blip24).publish().set(Blip24(1, Transform3d()))

    vision_nt_struct4 = inst.getStructArrayTopic("vision/blips", Blip24).publish()

#    t = Pose2d(1,2)
#    t2 = Pose2d(3,4)
    # t3 = Blip24(3, Pose2d(1,2,Rotation2d()))
    # t4 = Blip24(3, Pose2d(3,4,Rotation2d(1)))
    t3 = Blip24(3, Transform3d(1,2,3,Rotation3d()))
    t4 = Blip24(3, Pose2d(3,4,5,Rotation3d()))

    while True:

#        vision_nt_struct.set([t, t2])
#        vision_nt_struct2.set(t)
        vision_nt_struct4.set([t3, t4])
        # empty?
        # vision_nt_struct4.set([])

        time.sleep(0.1)

main()