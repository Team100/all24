package org.team100.lib.timing;

import static org.junit.jupiter.api.Assertions.assertEquals;

import org.junit.jupiter.api.Test;
import org.team100.lib.geometry.Pose2dWithMotion;
import org.team100.lib.motion.drivetrain.kinodynamics.SwerveKinodynamicsFactory;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Twist2d;

class YawRateConstraintTest {
    private static final double kDelta = 0.001;

    @Test
    void testSpin() {
        // one radian/m in place i.e. no constraint
        YawRateConstraint c = new YawRateConstraint(SwerveKinodynamicsFactory.forTest());
        Pose2dWithMotion p = new Pose2dWithMotion(
                new Pose2d(), new Twist2d(0, 0, 1), 0, 0);
        assertEquals(Double.NEGATIVE_INFINITY, c.getMinMaxAcceleration(p, 0).getMinAccel(), kDelta);
        assertEquals(Double.POSITIVE_INFINITY, c.getMinMaxAcceleration(p, 0).getMaxAccel(), kDelta);
        assertEquals(Double.MAX_VALUE, c.getMaxVelocity(p), kDelta);
    }

    @Test
    void testNormal() {
        // towards +x, 1 rad/m, 1 m/s wheel -> 1 rad/s limit => 2.8 m/s (which violates the linear constraint but it's ok)
        YawRateConstraint c = new YawRateConstraint(SwerveKinodynamicsFactory.forTest());
        Pose2dWithMotion p = new Pose2dWithMotion(
                new Pose2d(),
                new Twist2d(1, 0, 1), // spatial, so rad/m
                0, 0);
        assertEquals(Double.NEGATIVE_INFINITY, c.getMinMaxAcceleration(p, 0).getMinAccel(), kDelta);
        assertEquals(Double.POSITIVE_INFINITY, c.getMinMaxAcceleration(p, 0).getMaxAccel(), kDelta);
        assertEquals(2.828, c.getMaxVelocity(p), kDelta);
    }


    @Test
    void testNormal2() {
        // towards +x, 1 rad/m, 2 rad/s limit => 2 m/s
        YawRateConstraint c = new YawRateConstraint(SwerveKinodynamicsFactory.forTest2());
        Pose2dWithMotion p = new Pose2dWithMotion(
                new Pose2d(),
                new Twist2d(1, 0, 1), // spatial, so rad/m
                0, 0);
        assertEquals(Double.NEGATIVE_INFINITY, c.getMinMaxAcceleration(p, 0).getMinAccel(), kDelta);
        assertEquals(Double.POSITIVE_INFINITY, c.getMinMaxAcceleration(p, 0).getMaxAccel(), kDelta);
        assertEquals(5.656, c.getMaxVelocity(p), kDelta);
    }

}
