package org.team100.lib.util;

import static org.junit.jupiter.api.Assertions.assertEquals;

import java.io.IOException;

import org.junit.jupiter.api.Test;
import org.team100.lib.config.Camera;
import org.team100.lib.localization.PoseEstimationHelper;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;

class CameraUpdaterTest {
    @Test
    void testRotation() throws IOException {
        Pose3d tagInFieldCoords = new Pose3d();
        Transform3d aprilTagTransform = PoseEstimationHelper.getTransformFromRobotPose(Camera.TEST2.getOffset(),
                new Pose2d(1, tagInFieldCoords.getY(), new Rotation2d(Math.PI)), tagInFieldCoords);
        assertEquals(aprilTagTransform.getX(), 0, 0.0001);
        System.out.println("APRIL TAG TRANSFORM: " + aprilTagTransform);
    }

    @Test
    void testRotation2() {
        Pose3d tagInFieldCoords = new Pose3d();
        Transform3d aprilTagTransform = PoseEstimationHelper.getTransformFromRobotPose(Camera.TEST4.getOffset(),
                new Pose2d(1, tagInFieldCoords.getY(), new Rotation2d(Math.PI)), tagInFieldCoords);
        assertEquals(aprilTagTransform.getX(), 0, 0.0001);
        System.out.println("APRIL TAG TRANSFORM: " + aprilTagTransform);
    }

    @Test
    void testRotation3() {
        Pose3d tagInFieldCoords = new Pose3d();
        Transform3d aprilTagTransform = PoseEstimationHelper.getTransformFromRobotPose(Camera.TEST4.getOffset(),
                new Pose2d(1, tagInFieldCoords.getY(), new Rotation2d(Math.PI)), tagInFieldCoords);
        assertEquals(aprilTagTransform.getX(), 0, 0.0001);
        System.out.println("APRIL TAG TRANSFORM: " + aprilTagTransform);
    }
}
