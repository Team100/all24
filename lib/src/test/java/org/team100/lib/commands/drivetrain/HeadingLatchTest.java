package org.team100.lib.commands.drivetrain;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertNull;

import org.junit.jupiter.api.Test;
import org.team100.lib.geometry.GeometryUtil;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Twist2d;

public class HeadingLatchTest {
    private static final double kDelta = 0.001;

    @Test
    void testInit() {
        HeadingLatch l = new HeadingLatch();
        Rotation2d pov = null;
        Twist2d input = new Twist2d();
        Rotation2d desiredRotation = l.latchedRotation(pov, input);
        assertNull(desiredRotation);
    }

    @Test
    void testLatch() {
        HeadingLatch l = new HeadingLatch();
        Rotation2d pov = GeometryUtil.kRotation90;
        Twist2d input = new Twist2d();
        Rotation2d desiredRotation = l.latchedRotation(pov, input);
        assertEquals(Math.PI / 2, desiredRotation.getRadians(), kDelta);
        pov = null;
        desiredRotation = l.latchedRotation(pov, input);
        assertEquals(Math.PI / 2, desiredRotation.getRadians(), kDelta);
    }

    @Test
    void testUnLatch() {
        HeadingLatch l = new HeadingLatch();
        Rotation2d pov = GeometryUtil.kRotation90;
        Twist2d input = new Twist2d();
        Rotation2d desiredRotation = l.latchedRotation(pov, input);
        assertEquals(Math.PI / 2, desiredRotation.getRadians(), kDelta);
        pov = null;
        desiredRotation = l.latchedRotation(pov, input);
        assertEquals(Math.PI / 2, desiredRotation.getRadians(), kDelta);
        input = new Twist2d(0, 0, 1);
        desiredRotation = l.latchedRotation(pov, input);
        assertNull(desiredRotation);
    }

    @Test
    void testExplicitUnLatch() {
        HeadingLatch l = new HeadingLatch();
        Rotation2d pov = GeometryUtil.kRotation90;
        Twist2d input = new Twist2d();
        Rotation2d desiredRotation = l.latchedRotation(pov, input);
        assertEquals(Math.PI / 2, desiredRotation.getRadians(), kDelta);
        pov = null;
        desiredRotation = l.latchedRotation(pov, input);
        assertEquals(Math.PI / 2, desiredRotation.getRadians(), kDelta);
        l.unlatch();
        desiredRotation = l.latchedRotation(pov, input);
        assertNull(desiredRotation);
    }
}
