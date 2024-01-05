package org.team100.lib.motion.drivetrain.manual;

import static org.junit.jupiter.api.Assertions.assertEquals;

import org.junit.jupiter.api.Test;
import org.team100.lib.motion.drivetrain.kinodynamics.SwerveKinodynamics;
import org.team100.lib.motion.drivetrain.kinodynamics.SwerveKinodynamicsFactory;

import edu.wpi.first.math.geometry.Twist2d;

class ManualFieldRelativeSpeedsTest {
    private static final double kDelta = 0.001;

    @Test
    void testTwistZero() {
        SwerveKinodynamics limits = SwerveKinodynamicsFactory.forTest();
        ManualFieldRelativeSpeeds manual = new ManualFieldRelativeSpeeds(limits);
        Twist2d input = new Twist2d();
        Twist2d twist = manual.apply(input);
        assertEquals(0, twist.dx, kDelta);
        assertEquals(0, twist.dy, kDelta);
        assertEquals(0, twist.dtheta, kDelta);
    }

    @Test
    void testTwistNonzero() {
        SwerveKinodynamics limits = SwerveKinodynamicsFactory.forTest();
        ManualFieldRelativeSpeeds manual = new ManualFieldRelativeSpeeds(limits);
        // these inputs are clipped and desaturated
        Twist2d input = new Twist2d(1, 2, 3);
        Twist2d twist = manual.apply(input);
        assertEquals(0.223, twist.dx, kDelta);
        assertEquals(0.447, twist.dy, kDelta);
        assertEquals(1.414, twist.dtheta, kDelta);
    }

}
