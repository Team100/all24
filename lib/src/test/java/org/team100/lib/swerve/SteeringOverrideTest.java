package org.team100.lib.swerve;

import static org.junit.jupiter.api.Assertions.assertEquals;

import java.util.Optional;

import org.junit.jupiter.api.Test;
import org.team100.lib.geometry.GeometryUtil;
import org.team100.lib.motion.drivetrain.kinodynamics.SwerveKinodynamics;
import org.team100.lib.motion.drivetrain.kinodynamics.SwerveKinodynamicsFactory;
import org.team100.lib.motion.drivetrain.kinodynamics.SwerveModuleState100;
import org.team100.lib.logging.primitive.TestPrimitiveLogger;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.logging.TestLoggerFactory;

import edu.wpi.first.math.geometry.Rotation2d;

class SteeringOverrideTest {
    private static final double kDelta = 0.001;
    private static final LoggerFactory logger = new TestLoggerFactory(new TestPrimitiveLogger());

    @Test
    void testUnconstrained() {
        SwerveKinodynamics l = SwerveKinodynamicsFactory.forTest();
        SteeringOverride c = new SteeringOverride(logger, l);

        SwerveModuleState100[] desiredModuleStates = new SwerveModuleState100[] {
                new SwerveModuleState100(0, Optional.of(GeometryUtil.kRotationZero))
        };
        SwerveModuleState100[] prevModuleStates = new SwerveModuleState100[] {
                new SwerveModuleState100(0, Optional.of(GeometryUtil.kRotationZero))
        };
        Rotation2d[] overrideSteering = new Rotation2d[1];

        double s = c.overrideIfStopped(
                desiredModuleStates,
                prevModuleStates,
                overrideSteering);

        assertEquals(1.0, s, kDelta);
    }

    @Test
    void testConstrained() {
        SwerveKinodynamics l = SwerveKinodynamicsFactory.forTest2();
        SteeringOverride c = new SteeringOverride(logger, l);

        SwerveModuleState100[] desiredModuleStates = new SwerveModuleState100[] {
                new SwerveModuleState100(1, Optional.of(GeometryUtil.kRotation90))
        };
        SwerveModuleState100[] prevModuleStates = new SwerveModuleState100[] {
                new SwerveModuleState100(0, Optional.of(GeometryUtil.kRotationZero))
        };
        Rotation2d[] overrideSteering = new Rotation2d[1];

        double s = c.overrideIfStopped(
                desiredModuleStates,
                prevModuleStates,
                overrideSteering);

        // s = 0 stops the drive motors
        assertEquals(0, s, kDelta);
        assertEquals(1, overrideSteering.length);
        // limit is 1 radian per second, time step is 0.02 sec, so 0.02 radians
        assertEquals(0.02, overrideSteering[0].getRadians(), kDelta);
    }

}
