package org.team100.lib.controller;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.junit.jupiter.api.Test;
import org.team100.lib.motion.drivetrain.SwerveState;

import edu.wpi.first.math.geometry.Twist2d;

class FullStateDriveControllerTest {
    private static final double kDelta = 0.001;

    @Test
    void testAtRest() {
        FullStateDriveController c = new FullStateDriveController();
        assertFalse(c.atReference());
        Twist2d t = c.calculate(
                new SwerveState(
                        new State100(0, 0, 0),
                        new State100(0, 0, 0),
                        new State100(0, 0, 0)),
                new SwerveState(
                        new State100(0, 0, 0),
                        new State100(0, 0, 0),
                        new State100(0, 0, 0)));
        assertEquals(0, t.dx, kDelta);
        assertEquals(0, t.dy, kDelta);
        assertEquals(0, t.dtheta, kDelta);
        assertTrue(c.atReference());
    }

    @Test
    void testFar() {
        FullStateDriveController c = new FullStateDriveController();
        assertFalse(c.atReference());
        Twist2d t = c.calculate(
                new SwerveState(
                        new State100(0, 0, 0),
                        new State100(0, 0, 0),
                        new State100(0, 0, 0)),
                new SwerveState(
                        new State100(1, 0, 0),
                        new State100(0, 0, 0),
                        new State100(0, 0, 0)));
        // 1m error so dx should be K*e = 1
        assertEquals(1, t.dx, kDelta);
        assertEquals(0, t.dy, kDelta);
        assertEquals(0, t.dtheta, kDelta);
        assertFalse(c.atReference());
    }

    @Test
    void testFast() {
        FullStateDriveController c = new FullStateDriveController();
        assertFalse(c.atReference());
        Twist2d t = c.calculate(
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
        assertEquals(2, t.dx, kDelta);
        assertEquals(0, t.dy, kDelta);
        assertEquals(0, t.dtheta, kDelta);
        assertFalse(c.atReference());
    }

    @Test
    void testOnTrack() {
        FullStateDriveController c = new FullStateDriveController();
        assertFalse(c.atReference());
        Twist2d t = c.calculate(
                new SwerveState(
                        new State100(0, 0, 0),
                        new State100(0, 0, 0),
                        new State100(0, 0, 0)),
                new SwerveState(
                        new State100(-1, 0.5, 0),
                        new State100(0, 0, 0),
                        new State100(0, 0, 0)));
        // position and velocity controls are opposite, so just cruise
        assertEquals(0, t.dx, kDelta);
        assertEquals(0, t.dy, kDelta);
        assertEquals(0, t.dtheta, kDelta);
        assertFalse(c.atReference());
    }

        @Test
    void testAllAxes() {
        FullStateDriveController c = new FullStateDriveController();
        assertFalse(c.atReference());
        Twist2d t = c.calculate(
                new SwerveState(
                        new State100(0, 0, 0),
                        new State100(0, 0, 0),
                        new State100(0, 0, 0)),
                new SwerveState(
                        new State100(1, 0, 0),
                        new State100(2, 0, 0),
                        new State100(3, 0, 0)));
        // 1m error so dx should be K*e = 1
        assertEquals(1, t.dx, kDelta);
        assertEquals(2, t.dy, kDelta);
        assertEquals(3, t.dtheta, kDelta);
        assertFalse(c.atReference());
    }

}
