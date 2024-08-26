# pylint: disable=no-member

# can i get the numerical derivative c++ code to work via python?

import gtsam
import numpy as np

x1 = gtsam.Point2(0,0)
print(type(x1)) # this is ndarray because Point2 is really Eigen::Vector2d
x2 = gtsam.Point2(1,1)
d = np.linalg.norm(x1-x2)
print(d)


