package org.team100.lib.swerve;

import static org.junit.jupiter.api.Assertions.assertEquals;

import org.junit.jupiter.api.Test;
import org.team100.lib.motion.drivetrain.kinodynamics.SwerveKinodynamics;
import org.team100.lib.motion.drivetrain.kinodynamics.SwerveKinodynamicsFactory;
import org.team100.lib.telemetry.SupplierLogger;
import org.team100.lib.telemetry.TestLogger;

class DriveAccelerationLimiterTest {
    private static final double kDelta = 0.001;
    private static final SupplierLogger logger = new TestLogger().getSupplierLogger();

    @Test
    void testUnconstrained() {
        SwerveKinodynamics l = SwerveKinodynamicsFactory.forTest(logger);
        DriveAccelerationLimiter c = new DriveAccelerationLimiter(logger, l);
        double[] prev_vx = new double[] { 0 };
        double[] prev_vy = new double[] { 0 };
        double[] desired_vx = new double[] { 0 };
        double[] desired_vy = new double[] { 0 };
        double s = c.enforceWheelAccelLimit(
                prev_vx,
                prev_vy,
                desired_vx,
                desired_vy,
                0.02);
        assertEquals(1, s, kDelta);
    }

    @Test
    void testConstrained() {
        SwerveKinodynamics l = SwerveKinodynamicsFactory.forTest(logger);
        DriveAccelerationLimiter c = new DriveAccelerationLimiter(logger, l);
        double[] prev_vx = new double[] { 0 };
        double[] prev_vy = new double[] { 0 };
        double[] desired_vx = new double[] { 1 };
        double[] desired_vy = new double[] { 0 };
        double s = c.enforceWheelAccelLimit(
                prev_vx,
                prev_vy,
                desired_vx,
                desired_vy,
                0.02);
        assertEquals(0.02, s, kDelta);
    }
}
