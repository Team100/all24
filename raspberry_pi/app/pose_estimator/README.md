# GTSAM

Georgia Tech Smoothing And Mapping (GTSAM) is a framework for "factor graph" computations, which uses iterative linearization and LM solving for high-dimensionality problems in robotics.

We use GTSAM to solve two problems:

* __Incremental sensor fusion ("smoothing").__  Using recent camera, gyro, and odometry measurements, find the current robot pose, and the uncertainty in that pose.  This is a more sophisticated version of the WPILib "Pose Estimator".

* __Camera alignment and calibration.__  With many cameras, it is a pain to get all the offsets and distortion measurements just right.  Instead, we can provide a lot of input using camera, gyro, and odometry, and GTSAM can solve for offset and distortion consistent with the gyro and odometry measurements.

For more on the design of the Team 100 GTSAM applications, see the [design doc](https://docs.google.com/document/d/1z58ovdQ_rDQaW90we5c4xnuo2mpjdJYAjIfBhLHy2UQ/edit).
