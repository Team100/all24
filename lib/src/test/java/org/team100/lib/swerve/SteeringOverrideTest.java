package org.team100.lib.swerve;

import static org.junit.jupiter.api.Assertions.assertEquals;

import org.junit.jupiter.api.Test;
import org.team100.lib.geometry.GeometryUtil;
import org.team100.lib.motion.drivetrain.kinodynamics.SwerveKinodynamics;
import org.team100.lib.motion.drivetrain.kinodynamics.SwerveKinodynamicsFactory;
import org.team100.lib.motion.drivetrain.kinodynamics.SwerveModuleState100;

import edu.wpi.first.math.geometry.Rotation2d;

class SteeringOverrideTest {
    private static final double kDelta = 0.001;

    @Test
    void testUnconstrained() {
        SwerveKinodynamics l = SwerveKinodynamicsFactory.forTest();
        SteeringOverride c = new SteeringOverride("foo", l);

        SwerveModuleState100[] desiredModuleStates = new SwerveModuleState100[] {
                new SwerveModuleState100(0, GeometryUtil.kRotationZero)
        };
        SwerveModuleState100[] prevModuleStates = new SwerveModuleState100[] {
                new SwerveModuleState100(0, GeometryUtil.kRotationZero)
        };
        Rotation2d[] overrideSteering = new Rotation2d[1];

        double s = c.overrideIfStopped(
                desiredModuleStates,
                prevModuleStates,
                overrideSteering,
                0.02);

        assertEquals(1.0, s, kDelta);
    }

    @Test
    void testConstrained() {
        SwerveKinodynamics l = SwerveKinodynamicsFactory.forTest2();
        SteeringOverride c = new SteeringOverride("foo", l);

        SwerveModuleState100[] desiredModuleStates = new SwerveModuleState100[] {
                new SwerveModuleState100(1, GeometryUtil.kRotation90)
        };
        SwerveModuleState100[] prevModuleStates = new SwerveModuleState100[] {
                new SwerveModuleState100(0, GeometryUtil.kRotationZero)
        };
        Rotation2d[] overrideSteering = new Rotation2d[1];

        double s = c.overrideIfStopped(
                desiredModuleStates,
                prevModuleStates,
                overrideSteering,
                0.02);

        // s = 0 stops the drive motors
        assertEquals(0, s, kDelta);
        assertEquals(1, overrideSteering.length);
        // limit is 1 radian per second, time step is 0.02 sec, so 0.02 radians
        assertEquals(0.02, overrideSteering[0].getRadians(), kDelta);
    }

}
