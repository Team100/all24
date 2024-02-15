# Calibration

Joel did a round of calibration of the raspberry pi cameras v2 and v3 wide, using the scripts called calibration_recorder.py (to save some images), calibrate.py (to calculate the coefficients), and undistort.py (to view the results), yielding these results:

## V2 Camera

```
{'camera_matrix': {'cx': 421.5895522145918,
               	'cy': 318.3343004658085,
               	'fx': 657.9975417150848,
               	'fy': 660.1629265845845},
 'distance_calibration-board_to_camera_mean_m': 0.40868474065165356,
 'distance_calibration-board_to_camera_stddev_m': 0.04476317642201338,
 'distance_calibration-board_to_camera_variance_m': 0.0020037419633882943,
 'distortion': {'k1': 0.022676772278455,
            	'k2': 39.27926566019575,
            	'k3': -65.9779907190678,
            	'p1': 0.000534833047305098,
            	'p2': -0.0017694920062869132},
 'distortion_matrix': array([[ 2.26767723e-02,  3.92792657e+01,  5.34833047e-04,
    	-1.76949201e-03, -6.59779907e+01, -5.75883422e-02,
     	3.81831051e+01, -6.37029103e+01,  0.00000000e+00,
     	0.00000000e+00,  0.00000000e+00,  0.00000000e+00,
     	0.00000000e+00,  0.00000000e+00]]),
 'horizontal_fov': 64.60388467629332,
 'horizontal_shift': 0.006718211796384347,
 'n_max_frames_found_corners': 114.0,
 'rms_re-projection_error': 0.23647283168381936,
 'vertical_fov': 50.0229489011369,
 'vertical_shift': 0.016776461795143662}
```

## V3 Wide Camera

```
{'camera_matrix': {'cx': 578.3704209176427,
               	'cy': 328.00498435183516,
               	'fx': 496.9138631279756,
               	'fy': 498.6489729088972},
 'distance_calibration-board_to_camera_mean_m': 0.3968908060351491,
 'distance_calibration-board_to_camera_stddev_m': 0.13316677774375849,
 'distance_calibration-board_to_camera_variance_m': 0.017733390694655567,
 'distortion': {'k1': -1.1834127905420881,
            	'k2': 0.7134539898088045,
            	'k3': -0.002945290844279676,
            	'p1': 0.0007902041630546306,
            	'p2': -0.0007388798560729055},
 'distortion_matrix': array([[-1.18341279e+00,  7.13453990e-01,  7.90204163e-04,
    	-7.38879856e-04, -2.94529084e-03, -1.14073111e+00,
     	6.16356154e-01,  5.86094708e-02,  0.00000000e+00,
     	0.00000000e+00,  0.00000000e+00,  0.00000000e+00,
     	0.00000000e+00,  0.00000000e+00]]),
 'horizontal_fov': 79.8699314896507,
 'horizontal_shift': 0.19515675591062825,
 'n_max_frames_found_corners': 240.0,
 'rms_re-projection_error': 0.19604692892144363,
 'vertical_fov': 63.404677225730545,
 'vertical_shift': 0.03247562394778435}
```
