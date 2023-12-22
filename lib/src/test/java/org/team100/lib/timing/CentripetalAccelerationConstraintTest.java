package org.team100.lib.timing;

import static org.junit.jupiter.api.Assertions.assertEquals;

import org.junit.jupiter.api.Test;
import org.team100.lib.geometry.Pose2dWithMotion;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Twist2d;

 class CentripetalAccelerationConstraintTest {
        private static final double kDelta = 0.001;

    @Test
    void testSimple() {
        // 1 rad/m curve, 1 m/s^2 limit => 1 m/s
        CentripetalAccelerationConstraint c = new CentripetalAccelerationConstraint(1);
         Pose2dWithMotion p = new Pose2dWithMotion(
                new Pose2d(), new Twist2d(0, 0, 0), 1, 0);
        assertEquals(Double.NEGATIVE_INFINITY, c.getMinMaxAcceleration(p, 0).getMinAccel(), kDelta);
        assertEquals(Double.POSITIVE_INFINITY, c.getMinMaxAcceleration(p, 0).getMaxAccel(), kDelta);
        assertEquals(1, c.getMaxVelocity(p), kDelta);
    }

        @Test
    void testSimple2() {
        // 1 rad/m curve, 2 m/s^2 limit => 1.414 m/s
        CentripetalAccelerationConstraint c = new CentripetalAccelerationConstraint(2);
         Pose2dWithMotion p = new Pose2dWithMotion(
                new Pose2d(), new Twist2d(0, 0, 0), 1, 0);
        assertEquals(Double.NEGATIVE_INFINITY, c.getMinMaxAcceleration(p, 0).getMinAccel(), kDelta);
        assertEquals(Double.POSITIVE_INFINITY, c.getMinMaxAcceleration(p, 0).getMaxAccel(), kDelta);
        assertEquals(1.414, c.getMaxVelocity(p), kDelta);
    }
    
}
