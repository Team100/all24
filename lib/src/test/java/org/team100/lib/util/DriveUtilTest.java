package org.team100.lib.util;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.junit.jupiter.api.Test;
import org.team100.lib.hid.DriverControl;
import org.team100.lib.motion.drivetrain.kinodynamics.SwerveDriveKinematics100;
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
        // i'm not sure the module delta calculation is correct.
        Twist2d t = new Twist2d();
        Translation2d m_fl = new Translation2d(12, 12);
        Translation2d m_fr = new Translation2d(12, -12);
        Translation2d m_bl = new Translation2d(-12, 12);
        Translation2d m_br = new Translation2d(-12, -12);

        SwerveDriveKinematics100 m_kinematics = new SwerveDriveKinematics100(m_fl, m_fr, m_bl, m_br);

        SwerveModulePosition100[] p = m_kinematics.toSwerveModulePosition(t);

        assertEquals(0, p[0].distanceMeters, kDelta);
        assertTrue(p[0].angle.isEmpty());

    }

}
