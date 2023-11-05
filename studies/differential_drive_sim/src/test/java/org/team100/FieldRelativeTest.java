package org.team100;

import static org.junit.jupiter.api.Assertions.assertEquals;

import org.junit.jupiter.api.Test;

import frc.robot.commands.FieldRelativeDrive;
import frc.robot.commands.FieldRelativeDrive.Control;

public class FieldRelativeTest {
    @Test
    void testRotation0() {
        // pointing down the x axis
        double headingDeg = 0;
        double stickY = 1; // pushing down the x axis which should be fwd
        double stickX = 0;
        Control control = FieldRelativeDrive.rotate(headingDeg, stickY, stickX);
        assertEquals(1, control.x, 0.001);
        assertEquals(0, control.y, 0.001);
    }

    @Test
    void testRotation0X() {
        // pointing down the x axis
        double headingDeg = 0;
        double stickY = 0;
        double stickX = -1; // turn right
        Control control = FieldRelativeDrive.rotate(headingDeg, stickY, stickX);
        assertEquals(0, control.x, 0.001);
        assertEquals(-1, control.y, 0.001);
    }

    @Test
    void testRotation90() {
        // counterclockwise positive, pointing down the y axis
        double headingDeg = 90;
        double stickY = 1; // pushing down the x axis, this now means "turn right"
        double stickX = 0;
        Control control = FieldRelativeDrive.rotate(headingDeg, stickY, stickX);
        assertEquals(0, control.x, 0.001);
        assertEquals(-1, control.y, 0.001);
    }

    @Test
    void testRotation90X() {
        // counterclockwise positive, pointing down the y axis
        double headingDeg = 90;
        double stickY = 0;
        double stickX = -1; // back up
        Control control = FieldRelativeDrive.rotate(headingDeg, stickY, stickX);
        assertEquals(-1, control.x, 0.001);
        assertEquals(0, control.y, 0.001);
    }

    @Test
    void testRotationNegative90() {
        // counterclockwise positive, pointing down the negative y axis
        double headingDeg = -90;
        double stickY = 1; // pushing down the x axis, this now means "turn left"
        double stickX = 0;
        Control control = FieldRelativeDrive.rotate(headingDeg, stickY, stickX);
        assertEquals(0, control.x, 0.001);
        assertEquals(1, control.y, 0.001);
    }

    @Test
    void testRotation45() {
        // along y=x
        double headingDeg = 45;
        double stickY = 1; // pushing down the x axis, this now means "veer right"
        double stickX = 0;
        Control control = FieldRelativeDrive.rotate(headingDeg, stickY, stickX);
        assertEquals(0.707, control.x, 0.001);
        assertEquals(-0.707, control.y, 0.001);
    }

    @Test
    void testRotation45X() {
        // along y=x
        double headingDeg = 45;
        double stickY = 0;
        double stickX = -1;
        Control control = FieldRelativeDrive.rotate(headingDeg, stickY, stickX);
        assertEquals(-0.707, control.x, 0.001);
        assertEquals(-0.707, control.y, 0.001);
    }
}
