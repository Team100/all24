package org.team100.lib.timing;

import static org.junit.jupiter.api.Assertions.assertEquals;

import org.junit.jupiter.api.Test;
import org.team100.lib.geometry.Pose2dWithMotion;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Twist2d;

class YawRateConstraintTest {
    private static final double kDelta = 0.001;

    @Test
    void testSpin() {
        // one radian/m in place i.e. no constraint
        YawRateConstraint c = new YawRateConstraint(1);
        Pose2dWithMotion p = new Pose2dWithMotion(
                new Pose2d(), new Twist2d(0, 0, 1), 0, 0);
        assertEquals(Double.NEGATIVE_INFINITY, c.getMinMaxAcceleration(p, 0).min_acceleration(), kDelta);
        assertEquals(Double.POSITIVE_INFINITY, c.getMinMaxAcceleration(p, 0).max_acceleration(), kDelta);
        assertEquals(Double.MAX_VALUE, c.getMaxVelocity(p), kDelta);
    }

    @Test
    void testNormal() {
        // towards +x, 1 rad/m, 1 rad/s limit => 1 m/s
        YawRateConstraint c = new YawRateConstraint(1);
        Pose2dWithMotion p = new Pose2dWithMotion(
                new Pose2d(),
                new Twist2d(1, 0, 1), // spatial, so rad/m
                0, 0);
        assertEquals(Double.NEGATIVE_INFINITY, c.getMinMaxAcceleration(p, 0).min_acceleration(), kDelta);
        assertEquals(Double.POSITIVE_INFINITY, c.getMinMaxAcceleration(p, 0).max_acceleration(), kDelta);
        assertEquals(1, c.getMaxVelocity(p), kDelta);
    }


    @Test
    void testNormal2() {
        // towards +x, 1 rad/m, 2 rad/s limit => 2 m/s
        YawRateConstraint c = new YawRateConstraint(2);
        Pose2dWithMotion p = new Pose2dWithMotion(
                new Pose2d(),
                new Twist2d(1, 0, 1), // spatial, so rad/m
                0, 0);
        assertEquals(Double.NEGATIVE_INFINITY, c.getMinMaxAcceleration(p, 0).min_acceleration(), kDelta);
        assertEquals(Double.POSITIVE_INFINITY, c.getMinMaxAcceleration(p, 0).max_acceleration(), kDelta);
        assertEquals(2, c.getMaxVelocity(p), kDelta);
    }

}
