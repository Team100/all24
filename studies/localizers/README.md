# Localizers

These are vision interpreters that I thought would be helpful but we've never used them. With the future adoption of GTSAM i don't see this becoming more relevant in the future.

The "triangulation" interpreter uses two targets in the same view, for perhaps-more-accurate fixes. It didn't seem to help much in reality, though.

The "fire control" interpreter just yields a robot-relative target pose, for direct visual servoing without localization.

VisionDataProvider24.java:

```java
//
    private void firingSolution(
            final String cameraSerialNumber,
            final Blip24[] blips,
            final Transform3d cameraInRobotCoordinates,
            Alliance alliance) {
        for (Blip24 blip : blips) {
            if ((blip.getId() == 7 && alliance == Alliance.Blue) ||
                    (blip.getId() == 5 && alliance == Alliance.Red)) {
                Translation2d translation2d = PoseEstimationHelper.toTarget(cameraInRobotCoordinates, blip)
                        .getTranslation().toTranslation2d();
                if (!Experiments.instance.enabled(Experiment.HeedVision))
                    continue;
                m_fireControl.accept(translation2d);
            }
        }
    }


    private void triangulate(
            final String cameraSerialNumber,
            Blip24[] blips,
            Transform3d cameraInRobotCoordinates,
            double frameTimeSec,
            Rotation2d gyroRotation,
            Alliance alliance) {
        // if multiple tags are in view, triangulate to get another (perhaps more
        // accurate) estimate
        for (int i = 0; i < blips.length - 1; i++) {
            Blip24 b0 = blips[i];
            for (int j = i + 1; j < blips.length; ++j) {
                Blip24 b1 = blips[j];

                Optional<Pose3d> tagInFieldCordsOptional0 = m_layout.getTagPose(alliance, b0.getId());
                Optional<Pose3d> tagInFieldCordsOptional1 = m_layout.getTagPose(alliance, b1.getId());

                if (!tagInFieldCordsOptional0.isPresent())
                    continue;
                if (!tagInFieldCordsOptional1.isPresent())
                    continue;

                Translation2d T0 = tagInFieldCordsOptional0.get().getTranslation().toTranslation2d();
                Translation2d T1 = tagInFieldCordsOptional1.get().getTranslation().toTranslation2d();

                // in camera frame
                Transform3d t0 = PoseEstimationHelper.blipToTransform(b0);
                Transform3d t1 = PoseEstimationHelper.blipToTransform(b1);

                // in robot frame
                Transform3d rf0 = t0.plus(cameraInRobotCoordinates);
                Transform3d rf1 = t1.plus(cameraInRobotCoordinates);

                // in 2d
                Translation2d tr0 = rf0.getTranslation().toTranslation2d();
                Translation2d tr1 = rf1.getTranslation().toTranslation2d();

                // rotations
                Rotation2d r0 = tr0.getAngle();
                Rotation2d r1 = tr1.getAngle();

                Translation2d X = TriangulationHelper.solve(T0, T1, r0, r1);
                Pose2d currentRobotinFieldCoords = new Pose2d(X, gyroRotation);

                if (!Experiments.instance.enabled(Experiment.HeedVision))
                    continue;

                if (lastRobotInFieldCoords != null) {
                    double distanceM = GeometryUtil.distance(lastRobotInFieldCoords, currentRobotinFieldCoords);
                    if (distanceM <= kVisionChangeToleranceMeters) {
                        // this hard limit excludes false positives, which were a bigger problem in 2023
                        // due to the coarse tag family used. in 2024 this might not be an issue.
                        latestTimeUs = RobotController.getFPGATime();
                        m_poseEstimator.put(
                                frameTimeSec,
                                currentRobotinFieldCoords,
                                stateStdDevs(),
                                visionMeasurementStdDevs(distanceM));
                    }
                }
                lastRobotInFieldCoords = currentRobotinFieldCoords;
            }
        }
    }
```

Experiment.java

```java
//
    /**
     * Use multiple april tags to triangulate position
     */
    Triangulate,
```
