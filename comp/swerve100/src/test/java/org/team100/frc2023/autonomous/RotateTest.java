package org.team100.frc2023.autonomous;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.junit.jupiter.api.Test;
import org.team100.lib.motion.drivetrain.HeadingInterface;
import org.team100.lib.motion.drivetrain.SpeedLimits;
import org.team100.lib.motion.drivetrain.SwerveDriveSubsystem;
import org.team100.lib.motion.drivetrain.SwerveDriveSubsystemInterface;
import org.team100.lib.motion.drivetrain.SwerveState;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;

/** Example of mock objects for testing. */
public class RotateTest {
    private static final double kDelta = 0.001;

    class MockHeading implements HeadingInterface {
        @Override
        public Rotation2d getHeadingNWU() {
            return new Rotation2d();
        }

        @Override
        public double getHeadingRateNWU() {
            return 0;
        }
    }

    class MockSwerveDriveSubsystem implements SwerveDriveSubsystemInterface {

        Pose2d pose = new Pose2d();
        double output = 0;
        boolean stopped = false;

        @Override
        public Pose2d getPose() {
            return pose;
        }

        @Override
        public void stop() {
            stopped = true;
        }

        @Override
        public void setDesiredState(SwerveState desiredState) {
            output = desiredState.theta().v();
        }

        @Override
        public void truncate() {
            stop();
        }

        @Override
        public SwerveDriveSubsystem get() {
            return null;
        }

        @Override
        public ChassisSpeeds speeds() {
            return new ChassisSpeeds();
        }
    }

    class MockTimer extends Timer {

        double time = 0;

        @Override
        public void reset() {
            time = 0;
        }

        @Override
        public double get() {
            return time;
        }

    }

    @Test
    public void testRotate() {
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
        assertEquals(0, swerveDriveSubsystem.output, kDelta);

        timer.time = 1;
        rotate.execute();
        assertEquals(0.5, rotate.refTheta.getX(), kDelta);
        assertEquals(1.0, swerveDriveSubsystem.output, kDelta);

        timer.time = 2;
        swerveDriveSubsystem.pose = new Pose2d(0, 0, new Rotation2d(1));
        rotate.execute();
        assertEquals(1.408, rotate.refTheta.getX(), kDelta);
        assertEquals(0.571, swerveDriveSubsystem.output, kDelta);

        timer.time = 3;
        swerveDriveSubsystem.pose = new Pose2d(0, 0, new Rotation2d(Math.PI));

        assertFalse(swerveDriveSubsystem.stopped);
        rotate.end(false);
        assertTrue(swerveDriveSubsystem.stopped);

    }
}
