package org.team100.lib.swerve;

import static org.junit.jupiter.api.Assertions.assertEquals;

import org.junit.jupiter.api.Test;
import org.team100.lib.geometry.GeometryUtil;
import org.team100.lib.motion.drivetrain.kinodynamics.SwerveKinodynamics;
import org.team100.lib.motion.drivetrain.kinodynamics.SwerveKinodynamicsFactory;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;

class SteeringRateLimiterTest {
    private static final double kDelta = 0.001;

    @Test
    void testUnconstrained() {
        SwerveKinodynamics l = SwerveKinodynamicsFactory.forTest();
        SteeringRateLimiter c = new SteeringRateLimiter("foo", l);

        SwerveModuleState[] desiredModuleStates = new SwerveModuleState[] {
                new SwerveModuleState(0, GeometryUtil.kRotationZero)
        };
        SwerveModuleState[] prevModuleStates = new SwerveModuleState[] {
                new SwerveModuleState(0, GeometryUtil.kRotationZero)
        };
        double[] prev_vx = new double[] { 0 };
        double[] prev_vy = new double[] { 0 };
        Rotation2d[] prev_heading = new Rotation2d[] { GeometryUtil.kRotationZero };
        double[] desired_vx = new double[] { 0 };
        double[] desired_vy = new double[] { 0 };
        Rotation2d[] desired_heading = new Rotation2d[] { GeometryUtil.kRotationZero };
        Rotation2d[] overrideSteering = new Rotation2d[1];

        double s = c.enforceSteeringLimit(
                desiredModuleStates,
                prevModuleStates,
                prev_vx,
                prev_vy,
                prev_heading,
                desired_vx,
                desired_vy,
                desired_heading,
                overrideSteering,
                0.02);

        assertEquals(1.0, s, kDelta);
    }

    @Test
    void testConstrained() {
        SwerveKinodynamics l = SwerveKinodynamicsFactory.forTest2();
        SteeringRateLimiter c = new SteeringRateLimiter("foo", l);

        SwerveModuleState[] desiredModuleStates = new SwerveModuleState[] {
                new SwerveModuleState(1, GeometryUtil.kRotation90)
        };
        SwerveModuleState[] prevModuleStates = new SwerveModuleState[] {
                new SwerveModuleState(0, GeometryUtil.kRotationZero)
        };
        double[] prev_vx = new double[] { 0 };
        double[] prev_vy = new double[] { 0 };
        Rotation2d[] prev_heading = new Rotation2d[] { GeometryUtil.kRotationZero };
        double[] desired_vx = new double[] { 0 };
        double[] desired_vy = new double[] { 1 };
        Rotation2d[] desired_heading = new Rotation2d[] { GeometryUtil.kRotation90 };
        Rotation2d[] overrideSteering = new Rotation2d[1];

        double s = c.enforceSteeringLimit(desiredModuleStates,
                prevModuleStates, prev_vx,
                prev_vy, prev_heading, desired_vx, desired_vy,
                desired_heading, overrideSteering, 0.02);

        // s = 0 stops the drive motors
        assertEquals(0, s, kDelta);
        assertEquals(1, overrideSteering.length);
        // limit is 1 radian per second, time step is 0.02 sec, so 0.02 radians
        assertEquals(0.02, overrideSteering[0].getRadians(), kDelta);
    }
}
