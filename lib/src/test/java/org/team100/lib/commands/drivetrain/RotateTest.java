package org.team100.lib.commands.drivetrain;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.junit.jupiter.api.Test;
import org.team100.lib.geometry.GeometryUtil;
import org.team100.lib.motion.drivetrain.MockSwerveDriveSubsystem;
import org.team100.lib.motion.drivetrain.kinodynamics.SwerveKinodynamics;
import org.team100.lib.motion.drivetrain.kinodynamics.SwerveKinodynamicsFactory;
import org.team100.lib.sensors.MockHeading;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

/** Example of mock objects for testing. */
class RotateTest {
    private static final double kDelta = 0.02;

    @Test
    void testRotate() {
        MockSwerveDriveSubsystem swerveDriveSubsystem = new MockSwerveDriveSubsystem();
        swerveDriveSubsystem.pose = GeometryUtil.kPoseZero;
        MockHeading heading = new MockHeading();
        SwerveKinodynamics swerveKinodynamics = SwerveKinodynamicsFactory.forTest();
        double targetAngle = Math.PI / 2;

        Rotate rotate = new Rotate(
                swerveDriveSubsystem,
                heading,
                swerveKinodynamics,
                targetAngle);

        rotate.initialize();

        assertEquals(0, rotate.refTheta.position, kDelta); // at start
        assertEquals(0, swerveDriveSubsystem.twist.dtheta, kDelta);

        for (int i = 0; i < 10; ++i) {
            rotate.execute();
        }
        assertEquals(0.169, rotate.refTheta.position, kDelta);
        assertEquals(2.291, swerveDriveSubsystem.twist.dtheta, kDelta);

        for (int i = 0; i < 10; ++i) {
            rotate.execute();
        }
        swerveDriveSubsystem.pose = new Pose2d(0, 0, new Rotation2d(1));
        rotate.execute();

        assertEquals(0.716, rotate.refTheta.position, kDelta);
        assertEquals(1.836, swerveDriveSubsystem.twist.dtheta, kDelta);

        for (int i = 0; i < 50; ++i) {
            rotate.execute();
        }
        swerveDriveSubsystem.pose = new Pose2d(0, 0, new Rotation2d(Math.PI));
        assertFalse(swerveDriveSubsystem.stopped);
        rotate.end(false);
        assertTrue(swerveDriveSubsystem.stopped);

    }
}
