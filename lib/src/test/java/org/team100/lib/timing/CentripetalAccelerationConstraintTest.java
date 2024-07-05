package org.team100.lib.timing;

import static org.junit.jupiter.api.Assertions.assertEquals;

import org.junit.jupiter.api.Test;
import org.team100.lib.geometry.Pose2dWithMotion;
import org.team100.lib.motion.drivetrain.kinodynamics.SwerveKinodynamicsFactory;
import org.team100.lib.telemetry.TestLogger;
import org.team100.lib.telemetry.Logger;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Twist2d;

class CentripetalAccelerationConstraintTest {
    private static final double kDelta = 0.001;
    private static final double kCentripetalScale = 1.0;
    private static final Logger logger = new TestLogger();

    @Test
    void testSimple() {
        assertEquals(8.166, SwerveKinodynamicsFactory.forTest(logger).getMaxCapsizeAccelM_S2(), kDelta);

        // 1 rad/m curve, 8 m/s^2 limit => 2.8 m/s
        CentripetalAccelerationConstraint c = new CentripetalAccelerationConstraint(
                SwerveKinodynamicsFactory.forTest(logger),
                kCentripetalScale);
        Pose2dWithMotion p = new Pose2dWithMotion(
                new Pose2d(), new Twist2d(0, 0, 0), 1, 0);
        assertEquals(Double.NEGATIVE_INFINITY, c.getMinMaxAcceleration(p, 0).getMinAccel(), kDelta);
        assertEquals(Double.POSITIVE_INFINITY, c.getMinMaxAcceleration(p, 0).getMaxAccel(), kDelta);
        assertEquals(2.857, c.getMaxVelocity(p).getValue(), kDelta);
    }

    @Test
    void testSimple2() {
        assertEquals(4.083, SwerveKinodynamicsFactory.forTest2(logger).getMaxCapsizeAccelM_S2(), kDelta);
        // 1 rad/m curve, 4 m/s^2 limit => 2 m/s
        CentripetalAccelerationConstraint c = new CentripetalAccelerationConstraint(
                SwerveKinodynamicsFactory.forTest2(logger),
                kCentripetalScale);
        Pose2dWithMotion p = new Pose2dWithMotion(
                new Pose2d(), new Twist2d(0, 0, 0), 1, 0);
        assertEquals(Double.NEGATIVE_INFINITY, c.getMinMaxAcceleration(p, 0).getMinAccel(), kDelta);
        assertEquals(Double.POSITIVE_INFINITY, c.getMinMaxAcceleration(p, 0).getMaxAccel(), kDelta);
        assertEquals(2.021, c.getMaxVelocity(p).getValue(), kDelta);
    }

}
