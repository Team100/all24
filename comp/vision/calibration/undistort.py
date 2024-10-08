""" Undistort Viewer
"""

import os
import numpy as np
import cv2
import matplotlib.pyplot as plt

print(os.getcwd())

# v2
# reprojection err of 0.24
# hfov = 64.6, vfov = 50.0
# datadir = "../../../../../CALIB/calib2/"
# mtx = np.array([[657.99754172,   0.        , 421.58955221],
#        [  0.        , 660.16292658, 318.33430047],
#        [  0.        ,   0.        ,   1.        ]])
# dist = np.array([[ 2.26767723e-02,  3.92792657e+01,  5.34833047e-04,
#         -1.76949201e-03, -6.59779907e+01, -5.75883422e-02,
#          3.81831051e+01, -6.37029103e+01,  0.00000000e+00,
#          0.00000000e+00,  0.00000000e+00,  0.00000000e+00,
#          0.00000000e+00,  0.00000000e+00]])

# v3
# these lead to a reprojection error of 0.19
# hfov = 79.9, vfov=63.4, which is smaller than the spec,
# but i think the spec counts a very fishy 5% at the edge which the
# calibration had a hard time even probing; we should mask it out.
datadir = "../../../../../CALIB/calib3/"
mtx = np.array([[496.91386313,   0.        , 578.37042092],
      [  0.        , 498.64897291, 328.00498435],
      [  0.        ,   0.        ,   1.        ]])
dist = np.array([[-1.18341279e+00,  7.13453990e-01,  7.90204163e-04,
       -7.38879856e-04, -2.94529084e-03, -1.14073111e+00,
        6.16356154e-01,  5.86094708e-02,  0.00000000e+00,
        0.00000000e+00,  0.00000000e+00,  0.00000000e+00,
        0.00000000e+00,  0.00000000e+00]])
images = np.array([datadir + f for f in os.listdir(datadir) if f.endswith(".png")])
print(images)

for im in images:
    print("=> Processing image {0}".format(im))

    img = cv2.imread(im)

    img2 = cv2.undistort(img, mtx, dist)

    f, axarr = plt.subplots(2,1)
    axarr[0].title.set_text("original")
    axarr[0].imshow(img)
    axarr[1].title.set_text("undistorted")
    axarr[1].imshow(img2)
    plt.tight_layout()
    plt.show()

