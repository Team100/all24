package org.team100.lib.swerve;

import static org.junit.jupiter.api.Assertions.assertEquals;

import org.junit.jupiter.api.Test;
import org.team100.lib.geometry.GeometryUtil;
import org.team100.lib.motion.drivetrain.kinodynamics.SwerveKinodynamics;
import org.team100.lib.motion.drivetrain.kinodynamics.SwerveKinodynamicsFactory;

import edu.wpi.first.math.kinematics.SwerveModuleState;

class DriveAccelerationLimiterTest {

    private static final double kDelta = 0.001;

    @Test
    void testUnconstrained() {
        SwerveKinodynamics l = SwerveKinodynamicsFactory.forTest();
        DriveAccelerationLimiter c = new DriveAccelerationLimiter(l);
        SwerveModuleState[] prevModuleStates = new SwerveModuleState[] {
                new SwerveModuleState(0, GeometryUtil.kRotationZero)
        };
        double[] prev_vx = new double[] { 0 };
        double[] prev_vy = new double[] { 0 };
        double[] desired_vx = new double[] { 0 };
        double[] desired_vy = new double[] { 0 };
        double s = c.enforceWheelAccelLimit(
                prevModuleStates,
                prev_vx,
                prev_vy,
                desired_vx,
                desired_vy,
                0.02);
        assertEquals(1, s, kDelta);
    }

    @Test
    void testConstrained() {
        SwerveKinodynamics l = SwerveKinodynamicsFactory.forTest();
        DriveAccelerationLimiter c = new DriveAccelerationLimiter(l);
        SwerveModuleState[] prevModuleStates = new SwerveModuleState[] {
                new SwerveModuleState(0, GeometryUtil.kRotationZero)
        };
        double[] prev_vx = new double[] { 0 };
        double[] prev_vy = new double[] { 0 };
        double[] desired_vx = new double[] { 1 };
        double[] desired_vy = new double[] { 0 };
        double s = c.enforceWheelAccelLimit(
                prevModuleStates,
                prev_vx,
                prev_vy,
                desired_vx,
                desired_vy,
                0.02);
        assertEquals(0.02, s, kDelta);
    }
}
