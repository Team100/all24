package org.team100.lib.commands.drivetrain.manual;
import static org.junit.jupiter.api.Assertions.assertEquals;

import org.junit.jupiter.api.Test;
import org.team100.lib.hid.DriverControl;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.logging.TestLoggerFactory;
import org.team100.lib.logging.primitive.TestPrimitiveLogger;
import org.team100.lib.motion.drivetrain.kinodynamics.SwerveKinodynamics;
import org.team100.lib.motion.drivetrain.kinodynamics.SwerveKinodynamicsFactory;
import org.team100.lib.motion.drivetrain.kinodynamics.SwerveModuleStates;


class SimpleManualModuleStatesTest {
    private static final double kDelta = 0.001;
    private static final LoggerFactory logger = new TestLoggerFactory(new TestPrimitiveLogger());

    @Test
    void testZero() {
        SwerveKinodynamics limits = SwerveKinodynamicsFactory.forTest();
        SimpleManualModuleStates s = new SimpleManualModuleStates(logger, limits);
        DriverControl.Velocity input = new DriverControl.Velocity(0, 0, 0);
        SwerveModuleStates ms = s.apply(input);
        assertEquals(0, ms.frontLeft().angle.get().getRadians(), kDelta);
        assertEquals(0, ms.frontRight().angle.get().getRadians(), kDelta);
        assertEquals(0, ms.rearLeft().angle.get().getRadians(), kDelta);
        assertEquals(0, ms.rearRight().angle.get().getRadians(), kDelta);

        assertEquals(0, ms.frontLeft().speedMetersPerSecond, kDelta);
        assertEquals(0, ms.frontRight().speedMetersPerSecond, kDelta);
        assertEquals(0, ms.rearLeft().speedMetersPerSecond, kDelta);
        assertEquals(0, ms.rearRight().speedMetersPerSecond, kDelta);
    }

    @Test
    void testAngle() {
        SwerveKinodynamics limits = SwerveKinodynamicsFactory.forTest();
        SimpleManualModuleStates s = new SimpleManualModuleStates(logger, limits);
        DriverControl.Velocity input = new DriverControl.Velocity(0, 0, 0.5);
        SwerveModuleStates ms = s.apply(input);
        assertEquals(Math.PI / 2, ms.frontLeft().angle.get().getRadians(), kDelta);
        assertEquals(Math.PI / 2, ms.frontRight().angle.get().getRadians(), kDelta);
        assertEquals(Math.PI / 2, ms.rearLeft().angle.get().getRadians(), kDelta);
        assertEquals(Math.PI / 2, ms.rearRight().angle.get().getRadians(), kDelta);

        assertEquals(0, ms.frontLeft().speedMetersPerSecond, kDelta);
        assertEquals(0, ms.frontRight().speedMetersPerSecond, kDelta);
        assertEquals(0, ms.rearLeft().speedMetersPerSecond, kDelta);
        assertEquals(0, ms.rearRight().speedMetersPerSecond, kDelta);
    }

    @Test
    void testDrive() {
        SwerveKinodynamics limits = SwerveKinodynamicsFactory.forTest();
        SimpleManualModuleStates s = new SimpleManualModuleStates(logger, limits);
        DriverControl.Velocity input = new DriverControl.Velocity(0.5, 0, 0);
        SwerveModuleStates ms = s.apply(input);
        assertEquals(0, ms.frontLeft().angle.get().getRadians(), kDelta);
        assertEquals(0, ms.frontRight().angle.get().getRadians(), kDelta);
        assertEquals(0, ms.rearLeft().angle.get().getRadians(), kDelta);
        assertEquals(0, ms.rearRight().angle.get().getRadians(), kDelta);

        assertEquals(0.5, ms.frontLeft().speedMetersPerSecond, kDelta);
        assertEquals(0.5, ms.frontRight().speedMetersPerSecond, kDelta);
        assertEquals(0.5, ms.rearLeft().speedMetersPerSecond, kDelta);
        assertEquals(0.5, ms.rearRight().speedMetersPerSecond, kDelta);
    }

}
