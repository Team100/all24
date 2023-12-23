package org.team100.lib.swerve;

import static org.junit.jupiter.api.Assertions.assertEquals;

import org.junit.jupiter.api.Test;

class CentripetalAccelerationLimiterTest {
    private static final double kDelta = 0.001;

    /** zero delta v => no constraint */
    @Test
    void testUnconstrained() {
        SwerveKinematicLimits l = new SwerveKinematicLimits(1, 1, 1, 1, 1);
        CentripetalAccelerationLimiter c = new CentripetalAccelerationLimiter(l);
        double[] prev_vx = new double[] { 1 };
        double[] prev_vy = new double[] { 1 };
        double[] desired_vx = new double[] { 1 };
        double[] desired_vy = new double[] { 1 };
        double s = c.enforceCentripetalLimit(prev_vx, prev_vy, desired_vx, desired_vy, 1);
        assertEquals(1, s, kDelta);
    }

    /**
     * total delta v is 1.414 m/s, limit is 1 m/s/s, time step is 0.02 so 0.02 m/s/s
     * per step, which is 0.014 of the way.
     */
    @Test
    void testConstrained() {
        SwerveKinematicLimits l = new SwerveKinematicLimits(1, 1, 1, 1, 1);
        CentripetalAccelerationLimiter c = new CentripetalAccelerationLimiter(l);
        double[] prev_vx = new double[] { 1 };
        double[] prev_vy = new double[] { 0 };
        double[] desired_vx = new double[] { 0 };
        double[] desired_vy = new double[] { 1 };
        double s = c.enforceCentripetalLimit(prev_vx, prev_vy, desired_vx, desired_vy, 1);
        assertEquals(0.014, s, kDelta);
    }
}
