package org.team100.lib.controller.drivetrain;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.junit.jupiter.api.Test;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.logging.TestLoggerFactory;
import org.team100.lib.logging.primitive.TestPrimitiveLogger;
import org.team100.lib.motion.drivetrain.SwerveState;
import org.team100.lib.motion.drivetrain.kinodynamics.FieldRelativeVelocity;
import org.team100.lib.state.State100;

class FullStateDriveControllerTest {
    private static final double kDelta = 0.001;
    private static final LoggerFactory logger = new TestLoggerFactory(new TestPrimitiveLogger());
    private static HolonomicFieldRelativeController.Log hlog = new HolonomicFieldRelativeController.Log(logger);


    @Test
    void testAtRest() {
        FullStateDriveController c = new FullStateDriveController(hlog);
        assertFalse(c.atReference());
        FieldRelativeVelocity t = c.calculate(
                new SwerveState(
                        new State100(0, 0, 0),
                        new State100(0, 0, 0),
                        new State100(0, 0, 0)),
                new SwerveState(
                        new State100(0, 0, 0),
                        new State100(0, 0, 0),
                        new State100(0, 0, 0)));
        assertEquals(0, t.x(), kDelta);
        assertEquals(0, t.y(), kDelta);
        assertEquals(0, t.theta(), kDelta);
        assertTrue(c.atReference());
    }

    @Test
    void testFar() {
        FullStateDriveController c = new FullStateDriveController(hlog);
        assertFalse(c.atReference());
        FieldRelativeVelocity t = c.calculate(
                new SwerveState(
                        new State100(0, 0, 0),
                        new State100(0, 0, 0),
                        new State100(0, 0, 0)),
                new SwerveState(
                        new State100(1, 0, 0),
                        new State100(0, 0, 0),
                        new State100(0, 0, 0)));
        // 1m error so dx should be K*e = 1
        assertEquals(4, t.x(), kDelta);
        assertEquals(0, t.y(), kDelta);
        assertEquals(0, t.theta(), kDelta);
        assertFalse(c.atReference());
    }

    @Test
    void testFast() {
        FullStateDriveController c = new FullStateDriveController(hlog);
        assertFalse(c.atReference());
        FieldRelativeVelocity t = c.calculate(
                new SwerveState(
                        new State100(0, 0, 0),
                        new State100(0, 0, 0),
                        new State100(0, 0, 0)),
                new SwerveState(
                        new State100(0, 1, 0),
                        new State100(0, 0, 0),
                        new State100(0, 0, 0)));
        // position err is zero but velocity error is 1 and feedforward is also 1 so dx
        // should be FF + K*e = 2
        assertEquals(1.25, t.x(), kDelta);
        assertEquals(0, t.y(), kDelta);
        assertEquals(0, t.theta(), kDelta);
        assertFalse(c.atReference());
    }

    @Test
    void testOnTrack() {
        FullStateDriveController c = new FullStateDriveController(hlog);
        assertFalse(c.atReference());
        FieldRelativeVelocity t = c.calculate(
                new SwerveState(
                        new State100(0, 0, 0),
                        new State100(0, 0, 0),
                        new State100(0, 0, 0)),
                new SwerveState(
                        new State100(-1, 0.5, 0),
                        new State100(0, 0, 0),
                        new State100(0, 0, 0)));
        // position and velocity controls are opposite, so just cruise
        assertEquals(-3.375, t.x(), kDelta);
        assertEquals(0, t.y(), kDelta);
        assertEquals(0, t.theta(), kDelta);
        assertFalse(c.atReference());
    }

    @Test
    void testAllAxes() {
        FullStateDriveController c = new FullStateDriveController(hlog);
        assertFalse(c.atReference());
        FieldRelativeVelocity t = c.calculate(
                new SwerveState(
                        new State100(0, 0, 0),
                        new State100(0, 0, 0),
                        new State100(0, 0, 0)),
                new SwerveState(
                        new State100(1, 0, 0),
                        new State100(2, 0, 0),
                        new State100(3, 0, 0)));
        // 1m error so dx should be K*e = 1
        assertEquals(4, t.x(), kDelta);
        assertEquals(8, t.y(), kDelta);
        assertEquals(12, t.theta(), kDelta);
        assertFalse(c.atReference());
    }

}
