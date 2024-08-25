# try to understand what logexp does.

import math
import gtsam

x1 = gtsam.Pose2(0, 0, 0)
print(x1)  # (0, 0, 0)
x2 = gtsam.Pose2.Expmap([0, 0, 0])
print(x2)  # (0, 0, 0)
x3 = gtsam.Pose2(1, 0, math.pi / 2)
print(x3)  # (1, 0, 1.57)
x4 = gtsam.Pose2.Expmap([1, 0, math.pi / 2])
print(x4)  # (0.63, 0.63, 1.57)
x5 = gtsam.Pose2.Expmap([4, 0, 2 * math.pi])
print(x5)  # approximately (0, 0, 0), so a full circle.
print(x5.translation())  # (0, 0, 0)
x6 = x5.between(x1)
print(x6)  # about (0, 0, 0), they're about the same
x7 = gtsam.Pose2.Logmap(x5)
print(x7)
x8 = gtsam.Pose2(1.0, 2.5, -math.pi/2)
print(x8)
x9 = x8.compose(gtsam.Pose2.Expmap([0.5, 0, 0.5]))
print(x9)