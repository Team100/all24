package org.team100.lib.swerve;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.junit.jupiter.api.Test;
import org.team100.lib.motion.drivetrain.kinodynamics.SwerveKinodynamics;
import org.team100.lib.motion.drivetrain.kinodynamics.SwerveKinodynamicsFactory;
import org.team100.lib.logging.TestLogger;
import org.team100.lib.logging.SupplierLogger;

class SwerveUtilTest {
    private static final double kDelta = 0.001;
    private static final SupplierLogger logger = new TestLogger().getSupplierLogger();

    @Test
    void testFindDriveMaxS0() {
        double x_0 = 0;
        double y_0 = 0;
        double x_1 = 1;
        double y_1 = 0;
        double max_deviation = 0.02;
        int max_iterations = 1000;

        // since f0 = f1, there's nothing to do.
        double s = SwerveUtil.findDriveMaxS(
                x_0, y_0,
                x_1, y_1,
                max_deviation, max_iterations);

        assertEquals(0.02, s, kDelta);
        // this is an impossible steering solution, but the steering
        // checker should catch that.
    }

    @Test
    void testFindDriveMaxS() {
        double x_0 = -1;
        double y_0 = 0;
        double x_1 = 1;
        double y_1 = 0;
        double max_deviation = 0.1;
        int max_iterations = 1000;

        // since f0 = f1, there's nothing to do.
        double s = SwerveUtil.findDriveMaxS(
                x_0, y_0,
                x_1, y_1,
                max_deviation, max_iterations);

        assertEquals(1, s, kDelta);
        // this is an impossible steering solution, but the steering
        // checker should catch that.
    }

    @Test
    void testFindDriveMaxS2() {
        double x_0 = 1;
        double y_0 = 0;
        double x_1 = 0;
        double y_1 = 1;
        double max_deviation = 0.1;
        int max_iterations = 1000;

        double s = SwerveUtil.findDriveMaxS(
                x_0, y_0,
                x_1, y_1,
                max_deviation, max_iterations);

        // since f0 = f1, the drive solution is to maintain speed.

        assertEquals(1, s, kDelta);
        // this is an impossible steering solution, but the steering
        // checker should catch that.
    }

    @Test
    void testFindDriveMaxS3() {
        double x_0 = 0.5;
        double y_0 = 0;
        double x_1 = 1;
        double y_1 = 0;
        double max_deviation = 0.1;
        int max_iterations = 1000;

        double s = SwerveUtil.findDriveMaxS(
                x_0, y_0,
                x_1, y_1,
                max_deviation, max_iterations);

        // max_deviation should apply here, 0.1 deviation is 20% of the way from 0.5 to
        // 1
        assertEquals(0.2, s, kDelta);
    }

    @Test
    void testFindDriveMaxS4() {
        double x_0 = 0;
        double y_0 = 0.5;
        double x_1 = 1;
        double y_1 = 0;
        double max_deviation = 0.1;
        int max_iterations = 1000;

        double s = SwerveUtil.findDriveMaxS(
                x_0, y_0,
                x_1, y_1,
                max_deviation, max_iterations);

        // the max deviation applies to the hypot so this looks for the
        // point where the line between (0,0.5) and (1,0) and the circle
        // of r=0.6 intersect, which is about 0.557 of the way along the line.
        // (draw it to convince yourself)

        assertEquals(0.557, s, kDelta);
        // this is an impossible steering solution, but the steering
        // checker should catch that.
    }

    @Test
    void testFindSteeringMaxS4() {
        // this corresponds to the case above
        double x_0 = 0;
        double y_0 = 0.5;
        double f_0 = Math.PI / 2;
        double x_1 = 1;
        double y_1 = 0;
        double f_1 = 0;
        double max_deviation = 0.1;
        int max_iterations = 1000;

        double s = SwerveUtil.findSteeringMaxS(
                x_0, y_0, f_0,
                x_1, y_1, f_1,
                max_deviation, max_iterations);

        // this is a 1.57 radian turning requirement with a step
        // of 0.1 radians which would be s=-0.063 if measured along a
        // circle, but s is measured along the line from
        // (0,0.5) to (1,0) so s=0.048
        assertEquals(0.048, s, kDelta);
    }

    @Test
    void testGetIsAccel() {
        // decelerating
        assertFalse(SwerveUtil.isAccel(1, 0, 0, 1));
        // accelerating
        assertTrue(SwerveUtil.isAccel(0.5, 0.5, 0, 1));
    }

    @Test
    void testGetMaxVelStep() {
        SwerveKinodynamics l = SwerveKinodynamicsFactory.lowAccelHighDecel(logger);
        // decelerating
        assertEquals(0.2, SwerveUtil.getMaxVelStep(l, 1, 0, 0, 1, 0.02), kDelta);
        // acccelerating
        assertEquals(0.02, SwerveUtil.getMaxVelStep(l, 0.5, 0.5, 0, 1, 0.02), kDelta);
    }

    @Test
    void testGetMaxVelStepConstrained() {
        SwerveKinodynamics l = SwerveKinodynamicsFactory.forTest(logger);
        assertEquals(0.02, SwerveUtil.getMaxVelStep(l, 0, 0, 1, 0, 0.02), kDelta);
    }

    @Test
    void testGetMaxVelStepWithVelocityDependentAccel() {
        // available acceleration is not always the max.
        // a motor without current limiting has a straight declining torque curve
        // a motor with current limiting has a constant torque curve for awhile
        // hm, how to get the motor model in here?
    }

}
