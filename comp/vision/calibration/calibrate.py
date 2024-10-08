""" Camera Calibration

This script is mostly based on this example:

https://github.com/yumashino/Camera-Calibration-with-ChArUco-Board

I used the "Charuco" board found here:

https://github.com/carlosmccosta/charuco_detector/blob/master/boards/vector_format/black_and_white/ChArUco__A4_210x297mm__5x7_Board__17_Markers__40mm_SquareSize__30mm_MarkerSize__DictionaryNumber_3_4X4__4_BitsMarkers.pdf

and I waved it around near each of the cameras, a v2 and a v3 wide.

"""

import os
import math
import pprint
import numpy as np
import cv2
import matplotlib.pyplot as plt

print(os.getcwd())

# datadir = "../../../../../CALIB/calib2/"
datadir = "../../../../../CALIB/calib3/"
images = np.array([datadir + f for f in os.listdir(datadir) if f.endswith(".png")])
print(images)


# fig = plt.figure()
# ax = fig.add_subplot(1,1,1)
# plt.imshow(cv2.imread(images[0]))
# plt.show()


aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
board_shape = (5, 7)
# measured
checker_size = 36
marker_size = 27
charuco_board = cv2.aruco.CharucoBoard(
    board_shape, checker_size, marker_size, aruco_dict
)
charuco_detector = cv2.aruco.CharucoDetector(charuco_board)


obj_points_all = []
img_points_all = []

for im in images:
    print("=> Processing image {0}".format(im))

    img = cv2.imread(im)
    # plt.imshow(img)
    # plt.show()

    charucoCorners, charucoIds, markerCorners, markerIds = charuco_detector.detectBoard(
        img
    )

    # img2 = cv2.aruco.drawDetectedMarkers(img.copy(), markerCorners, markerIds)
    # plt.imshow(img2)
    # plt.show()

    # img3 = cv2.aruco.drawDetectedCornersCharuco(
        # img.copy(), charucoCorners, charucoIds, (255, 0, 0)
    # )
    # plt.imshow(img3)
    # plt.show()

    if charucoIds is None:
        continue

    obj_point, imgPoint = charuco_board.matchImagePoints(charucoCorners, charucoIds)
    if obj_point.shape[0] > 8 and not None in obj_point:
        obj_points_all.append(obj_point)
        img_points_all.append(imgPoint)

n_frames_found_corners = len(obj_points_all)
print(f"Found charuco corners in {n_frames_found_corners} frames.\n")

camera_img_w = 832
camera_img_h = 616

print("Estimating camera parameters. This may take a while...")
# cv2.CALIB_RATIONAL_MODEL is for fisheyes, which the wide one kinda is?
# cv2.CALIB_THIN_PRISM_MODEL, no idea what this is.
# cv2.CALIB_TILTED_MODEL is for tilt/shift which we don't have
# cv2.CALIB_ZERO_TANGENT_DIST doesn't help
ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(
    obj_points_all,
    img_points_all,
    (camera_img_w, camera_img_h),
    None,
    None,
    flags=cv2.CALIB_RATIONAL_MODEL ,
)

# pprint.pprint(tvecs)

print(f"Re-projection RMS error: {ret:.4f}")

fov_x = 2 * math.atan2(camera_img_w, 2 * mtx[0, 0]) * 180 / math.pi
fov_y = 2 * math.atan2(camera_img_h, 2 * mtx[1, 1]) * 180 / math.pi
shift_x = (mtx[0, 2] - camera_img_w / 2) / camera_img_w
shift_y = (mtx[1, 2] - camera_img_h / 2) / camera_img_h

# Distance from the calibration board to the camera


distances = [np.linalg.norm(tvec) for tvec in tvecs]
# pprint.pprint(distances)
mean_distance = np.mean(distances) / 1000  # mm -> m
variance_distance = np.var(distances) / 1000000  # mm -> m
std_deviation_distance = np.sqrt(variance_distance)

results = {
    "rms_re-projection_error": ret,
    "n_max_frames_found_corners": float(n_frames_found_corners),
    "distance_calibration-board_to_camera_mean_m": float(mean_distance),
    "distance_calibration-board_to_camera_variance_m": float(variance_distance),
    "distance_calibration-board_to_camera_stddev_m": float(std_deviation_distance),
    "camera_matrix": {
        "fx": float(mtx[0, 0]),
        "fy": float(mtx[1, 1]),
        "cx": float(mtx[0, 2]),
        "cy": float(mtx[1, 2]),
    },
    "camera_matrix_matrix": mtx,
    "distortion": {
        "k1": float(dist[0][0]),
        "k2": float(dist[0][1]),
        "p1": float(dist[0][2]),
        "p2": float(dist[0][3]),
        "k3": float(dist[0][4]),
    },
    "distortion_matrix": dist,
    "horizontal_fov": float(fov_x),
    "vertical_fov": float(fov_y),
    "horizontal_shift": float(shift_x),
    "vertical_shift": float(shift_y),
}

pprint.pprint(results)
