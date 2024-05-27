package org.team100.lib.timing;

import static org.junit.jupiter.api.Assertions.assertEquals;

import org.junit.jupiter.api.Test;
import org.team100.lib.geometry.Pose2dWithMotion;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;

class VelocityLimitRegionConstraintTest {
    private static final double kDelta = 0.001;

    @Test
    void testOutside() {
        // towards +x, 1 rad/m, 1 rad/s limit => 1 m/s
        VelocityLimitRegionConstraint c = new VelocityLimitRegionConstraint(
                new Translation2d(), new Translation2d(1, 1), 1);
        Pose2dWithMotion p = new Pose2dWithMotion(
                new Pose2d(-1, -1, new Rotation2d()),
                new Twist2d(0, 0, 0), // spatial, so rad/m
                0, 0);
        assertEquals(Double.NEGATIVE_INFINITY, c.getMinMaxAcceleration(p, 0).getMinAccel(), kDelta);
        assertEquals(Double.POSITIVE_INFINITY, c.getMinMaxAcceleration(p, 0).getMaxAccel(), kDelta);
        assertEquals(Double.POSITIVE_INFINITY, c.getMaxVelocity(p).getValue(), kDelta);
    }

    @Test
    void testInside() {
        // towards +x, 1 rad/m, 1 rad/s limit => 1 m/s
        VelocityLimitRegionConstraint c = new VelocityLimitRegionConstraint(
                new Translation2d(), new Translation2d(1, 1), 1);
        Pose2dWithMotion p = new Pose2dWithMotion(
                new Pose2d(0.5, 0.5, new Rotation2d()),
                new Twist2d(0, 0, 0), // spatial, so rad/m
                0, 0);
        assertEquals(Double.NEGATIVE_INFINITY, c.getMinMaxAcceleration(p, 0).getMinAccel(), kDelta);
        assertEquals(Double.POSITIVE_INFINITY, c.getMinMaxAcceleration(p, 0).getMaxAccel(), kDelta);
        assertEquals(1, c.getMaxVelocity(p).getValue(), kDelta);
    }

}
