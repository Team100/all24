package org.team100.lib.commands.drivetrain.manual;

import static org.junit.jupiter.api.Assertions.assertEquals;

import org.junit.jupiter.api.Test;
import org.team100.lib.hid.DriverControl;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.logging.TestLoggerFactory;
import org.team100.lib.logging.primitive.TestPrimitiveLogger;
import org.team100.lib.motion.drivetrain.SwerveModel;
import org.team100.lib.motion.drivetrain.kinodynamics.SwerveKinodynamics;
import org.team100.lib.motion.drivetrain.kinodynamics.SwerveKinodynamicsFactory;

import edu.wpi.first.math.kinematics.ChassisSpeeds;

class ManualChassisSpeedsTest {
    private static final double kDelta = 0.001;
    private static final LoggerFactory logger = new TestLoggerFactory(new TestPrimitiveLogger());

    @Test
    void testChassisSpeedsZero() {
        SwerveKinodynamics limits = SwerveKinodynamicsFactory.forTest();
        ManualChassisSpeeds manual = new ManualChassisSpeeds(logger, limits);
        DriverControl.Velocity input = new DriverControl.Velocity(0, 0, 0);
        ChassisSpeeds speeds = manual.apply(new SwerveModel(), input);
        assertEquals(0, speeds.vxMetersPerSecond, kDelta);
        assertEquals(0, speeds.vyMetersPerSecond, kDelta);
        assertEquals(0, speeds.omegaRadiansPerSecond, kDelta);
    }

    @Test
    void testChassisSpeedsNonzero() {
        SwerveKinodynamics limits = SwerveKinodynamicsFactory.forTest();
        assertEquals(1, limits.getMaxDriveVelocityM_S(), kDelta);
        assertEquals(2.828, limits.getMaxAngleSpeedRad_S(), kDelta);
        ManualChassisSpeeds manual = new ManualChassisSpeeds(logger, limits);
        // clipping to the unit circle, then desaturating.
        DriverControl.Velocity input = new DriverControl.Velocity(1, 2, 3);
        ChassisSpeeds speeds = manual.apply(new SwerveModel(), input);
        assertEquals(0.223, speeds.vxMetersPerSecond, kDelta);
        assertEquals(0.447, speeds.vyMetersPerSecond, kDelta);
        assertEquals(1.414, speeds.omegaRadiansPerSecond, kDelta);
    }
}
