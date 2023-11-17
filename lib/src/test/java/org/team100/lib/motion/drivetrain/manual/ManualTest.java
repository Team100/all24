package org.team100.lib.motion.drivetrain.manual;

import static org.junit.jupiter.api.Assertions.assertEquals;

import org.junit.jupiter.api.Test;
import org.team100.lib.motion.drivetrain.SwerveState;

import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;

public class ManualTest {
    private static final double kDelta = 0.001;

    @Test
    void testSwerveStateZero() {
        ManualSwerveState manual = new ManualSwerveState(() -> new Twist2d());
        SwerveState state = manual.get();
        assertEquals(0, state.x().x(), kDelta);
        assertEquals(0, state.y().x(), kDelta);
        assertEquals(0, state.theta().x(), kDelta);
    }

    @Test
    void testSwerveStateNonzero() {
        ManualSwerveState manual = new ManualSwerveState(() -> new Twist2d(1, 2, 3));
        SwerveState state = manual.get();
        assertEquals(1, state.x().x(), kDelta);
        assertEquals(2, state.y().x(), kDelta);
        assertEquals(3, state.theta().x(), kDelta);
    }

    @Test
    void testTwistZero() {
        ManualFieldRelativeSpeeds manual = new ManualFieldRelativeSpeeds(() -> new Twist2d());
        Twist2d twist = manual.get();
        assertEquals(0, twist.dx, kDelta);
        assertEquals(0, twist.dy, kDelta);
        assertEquals(0, twist.dtheta, kDelta);
    }

    @Test
    void testTwistNonzero() {
        ManualFieldRelativeSpeeds manual = new ManualFieldRelativeSpeeds(() -> new Twist2d(1, 2, 3));
        Twist2d twist = manual.get();
        assertEquals(1, twist.dx, kDelta);
        assertEquals(2, twist.dy, kDelta);
        assertEquals(3, twist.dtheta, kDelta);
    }

    @Test
    void testChassisSpeedsZero() {
        ManualChassisSpeeds manual = new ManualChassisSpeeds(() -> new Twist2d());
        ChassisSpeeds speeds = manual.get();
        assertEquals(0, speeds.vxMetersPerSecond, kDelta);
        assertEquals(0, speeds.vyMetersPerSecond, kDelta);
        assertEquals(0, speeds.omegaRadiansPerSecond, kDelta);
    }

    @Test
    void testChassisSpeedsNonzero() {
        ManualChassisSpeeds manual = new ManualChassisSpeeds(() -> new Twist2d(1, 2, 3));
        ChassisSpeeds speeds = manual.get();
        assertEquals(1, speeds.vxMetersPerSecond, kDelta);
        assertEquals(2, speeds.vyMetersPerSecond, kDelta);
        assertEquals(3, speeds.omegaRadiansPerSecond, kDelta);
    }

    @Test
    void testModuleStatesZero() {
        ManualModuleStates manual = new ManualModuleStates(() -> new Twist2d());
        SwerveModuleState[] speeds = manual.get();
        SwerveModuleState speed = speeds[0];
        assertEquals(0, speed.speedMetersPerSecond, kDelta);
        assertEquals(0, speed.angle.getRadians(), kDelta);
    }

    @Test
    void testModuleStatesInDeadband() {
        ManualModuleStates manual = new ManualModuleStates(() -> new Twist2d(0.1, 0.1, 0));
        SwerveModuleState[] speeds = manual.get();
        SwerveModuleState speed = speeds[0];
        assertEquals(0, speed.speedMetersPerSecond, kDelta);
        assertEquals(0, speed.angle.getRadians(), kDelta);
    }

    @Test
    void testModuleStatesOutsideDeadband() {
        ManualModuleStates manual = new ManualModuleStates(() -> new Twist2d(0.5, 0.5, 0));
        SwerveModuleState[] speeds = manual.get();
        SwerveModuleState speed = speeds[0];
        assertEquals(0.634, speed.speedMetersPerSecond, kDelta);
        assertEquals(Math.PI / 4, speed.angle.getRadians(), kDelta);
    }
}
