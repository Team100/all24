package org.team100.lib.motion.drivetrain.manual;

import static org.junit.jupiter.api.Assertions.assertEquals;

import org.junit.jupiter.api.Test;
import org.team100.lib.motion.drivetrain.kinodynamics.SwerveKinodynamics;
import org.team100.lib.motion.drivetrain.kinodynamics.SwerveKinodynamicsFactory;

import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

class ManualChassisSpeedsTest {
    private static final double kDelta = 0.001;

    @Test
    void testChassisSpeedsZero() {
        SwerveKinodynamics limits = SwerveKinodynamicsFactory.forTest();
        ManualChassisSpeeds manual = new ManualChassisSpeeds("foo", limits);
        Twist2d input =  new Twist2d();
        ChassisSpeeds speeds = manual.apply(input);
        assertEquals(0, speeds.vxMetersPerSecond, kDelta);
        assertEquals(0, speeds.vyMetersPerSecond, kDelta);
        assertEquals(0, speeds.omegaRadiansPerSecond, kDelta);
    }

    @Test
    void testChassisSpeedsNonzero() {
        SwerveKinodynamics limits = SwerveKinodynamicsFactory.forTest();
        assertEquals(1, limits.getMaxDriveVelocityM_S(), kDelta);
        assertEquals(2.828, limits.getMaxAngleSpeedRad_S(), kDelta);
        ManualChassisSpeeds manual = new ManualChassisSpeeds("foo", limits);
        // clipping to the unit circle, then desaturating.
        Twist2d input = new Twist2d(1, 2, 3);
        ChassisSpeeds speeds = manual.apply(input);

        assertEquals(0.223, speeds.vxMetersPerSecond, kDelta);
        assertEquals(0.447, speeds.vyMetersPerSecond, kDelta);
        assertEquals(1.414, speeds.omegaRadiansPerSecond, kDelta);
    }
}
