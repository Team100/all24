package org.team100.lib.commands;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.junit.jupiter.api.Test;
import org.team100.lib.motion.drivetrain.SpeedLimits;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

/** Example of mock objects for testing. */
class RotateTest {
    private static final double kDelta = 0.001;

    @Test
    void testRotate() {
        MockSwerveDriveSubsystem swerveDriveSubsystem = new MockSwerveDriveSubsystem();
        swerveDriveSubsystem.pose = new Pose2d();
        MockHeading heading = new MockHeading();
        SpeedLimits speedLimits = new SpeedLimits(1, 1, 1, 1);
        MockTimer timer = new MockTimer();
        double targetAngle = Math.PI / 2;
        Rotate rotate = new Rotate(
                swerveDriveSubsystem,
                heading,
                speedLimits,
                timer,
                targetAngle);

        timer.time = 100; // initial time is not zero

        rotate.initialize();

        assertEquals(0, timer.time, kDelta); // now the timer is reset
        assertEquals(0, rotate.m_profile.start().getX(), kDelta);
        assertEquals(Math.PI / 2, rotate.m_profile.end().getX(), kDelta);
        assertEquals(2.571, rotate.m_profile.duration(), kDelta);

        rotate.execute();

        assertEquals(0, rotate.refTheta.getX(), kDelta); // at start
        assertEquals(0, swerveDriveSubsystem.twist.dtheta, kDelta);

        timer.time = 1;
        rotate.execute();
        assertEquals(0.5, rotate.refTheta.getX(), kDelta);
        assertEquals(2.75, swerveDriveSubsystem.twist.dtheta, kDelta);

        timer.time = 2;
        swerveDriveSubsystem.pose = new Pose2d(0, 0, new Rotation2d(1));
        rotate.execute();
        assertEquals(1.408, rotate.refTheta.getX(), kDelta);
        assertEquals(1.998, swerveDriveSubsystem.twist.dtheta, kDelta);

        timer.time = 3;
        swerveDriveSubsystem.pose = new Pose2d(0, 0, new Rotation2d(Math.PI));

        assertFalse(swerveDriveSubsystem.stopped);
        rotate.end(false);
        assertTrue(swerveDriveSubsystem.stopped);

    }
}
