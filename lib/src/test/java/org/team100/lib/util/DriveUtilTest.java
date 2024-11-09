package org.team100.lib.util;

import static org.junit.jupiter.api.Assertions.assertEquals;

import java.util.Optional;
import java.util.Random;

import org.junit.jupiter.api.Test;
import org.team100.lib.geometry.GeometryUtil;
import org.team100.lib.hid.DriverControl;
import org.team100.lib.motion.drivetrain.kinodynamics.SwerveDriveKinematics100;
import org.team100.lib.motion.drivetrain.kinodynamics.SwerveModuleDelta;
import org.team100.lib.motion.drivetrain.kinodynamics.SwerveModuleDeltas;
import org.team100.lib.motion.drivetrain.kinodynamics.SwerveModulePosition100;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;

class DriveUtilTest {
    private static final double kDelta = 0.001;

    @Test
    void testClampTwist() {
        {
            // zero is no-op
            DriverControl.Velocity input = new DriverControl.Velocity(0, 0, 0);
            DriverControl.Velocity actual = DriveUtil.clampTwist(input, 1);
            assertEquals(0, actual.x(), kDelta);
            assertEquals(0, actual.y(), kDelta);
            assertEquals(0, actual.theta(), kDelta);
        }
        {
            // clip to the unit circle.
            DriverControl.Velocity input = new DriverControl.Velocity(1, 1, 0);
            DriverControl.Velocity actual = DriveUtil.clampTwist(input, 1);
            assertEquals(0.707, actual.x(), kDelta);
            assertEquals(0.707, actual.y(), kDelta);
        }
        {
            // leave the inside alone
            DriverControl.Velocity input = new DriverControl.Velocity(0.5, 0.5, 0);
            DriverControl.Velocity actual = DriveUtil.clampTwist(input, 1);
            assertEquals(0.5, actual.x(), kDelta);
            assertEquals(0.5, actual.y(), kDelta);
        }
    }

    @Test
    void testRoundTripModuleDeltas() {
        SwerveDriveKinematics100 m_kinematics = new SwerveDriveKinematics100(
                new Translation2d(0.5, 0.5),
                new Translation2d(0.5, -0.5),
                new Translation2d(-0.5, 0.5),
                new Translation2d(-0.5, -0.5));

        {
            // straight diagonal path
            Twist2d t = new Twist2d(1, 1, 0);
            SwerveModuleDeltas p = m_kinematics.toSwerveModuleDelta(t);
            Twist2d t2 = m_kinematics.toTwist2d(p);
            assertEquals(t, t2);
        }
        {
            // turning and moving
            Twist2d t = new Twist2d(1, 1, 1);
            SwerveModuleDeltas p = m_kinematics.toSwerveModuleDelta(t);
            Twist2d t2 = m_kinematics.toTwist2d(p);
            assertEquals(t, t2);
        }
        Random random = new Random(0);
        for (int i = 0; i < 500; ++i) {
            // inverse always works
            Twist2d t = new Twist2d(
                    random.nextDouble(),
                    random.nextDouble(),
                    random.nextDouble());
            SwerveModuleDeltas p = m_kinematics.toSwerveModuleDelta(t);
            Twist2d t2 = m_kinematics.toTwist2d(p);
            assertEquals(t, t2);
        }
    }

    @Test
    void testOneModule() {
        {
            SwerveModulePosition100 start = new SwerveModulePosition100(
                    0, Optional.of(GeometryUtil.kRotationZero));
            SwerveModulePosition100 end = new SwerveModulePosition100(
                    0, Optional.of(GeometryUtil.kRotationZero));
            SwerveModuleDelta delta = DriveUtil.delta(start, end);
            assertEquals(0, delta.distanceMeters, kDelta);
            assertEquals(0, delta.angle.get().getRadians(), kDelta);
        }
        {
            SwerveModulePosition100 start = new SwerveModulePosition100(
                    0, Optional.of(GeometryUtil.kRotationZero));
            SwerveModulePosition100 end = new SwerveModulePosition100(
                    1, Optional.of(GeometryUtil.kRotationZero));
            SwerveModuleDelta delta = DriveUtil.delta(start, end);
            assertEquals(1, delta.distanceMeters, kDelta);
            assertEquals(0, delta.angle.get().getRadians(), kDelta);
        }
        {
            // this ignores the initial zero rotation, and acts as if
            // the path is at 90 degrees the whole time.
            SwerveModulePosition100 start = new SwerveModulePosition100(
                    0, Optional.of(GeometryUtil.kRotationZero));
            SwerveModulePosition100 end = new SwerveModulePosition100(
                    1, Optional.of(GeometryUtil.kRotation90));
            SwerveModuleDelta delta = DriveUtil.delta(start, end);
            assertEquals(1, delta.distanceMeters, kDelta);
            assertEquals(1.571, delta.angle.get().getRadians(), kDelta);
        }
    }

}
