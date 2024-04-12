package org.team100.lib.commands.drivetrain;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertNull;

import org.junit.jupiter.api.Test;
import org.team100.lib.experiments.Experiment;
import org.team100.lib.experiments.Experiments;
import org.team100.lib.geometry.GeometryUtil;
import org.team100.lib.hid.DriverControl;

import edu.wpi.first.math.geometry.Rotation2d;

class HeadingLatchTest {
    private static final double kDelta = 0.001;

    @Test
    void testInit() {
        Experiments.instance.testOverride(Experiment.StickyHeading, false);
        HeadingLatch l = new HeadingLatch();
        Rotation2d currentRotation = new Rotation2d();
        Rotation2d pov = null;
        DriverControl.Velocity input = new DriverControl.Velocity(0, 0, 0);
        Rotation2d desiredRotation = l.latchedRotation(currentRotation, pov, input);
        assertNull(desiredRotation);
    }

    @Test
    void testLatch() {
        Experiments.instance.testOverride(Experiment.StickyHeading, false);
        HeadingLatch l = new HeadingLatch();
        Rotation2d currentRotation = new Rotation2d();
        Rotation2d pov = GeometryUtil.kRotation90;
        DriverControl.Velocity input = new DriverControl.Velocity(0, 0, 0);
        Rotation2d desiredRotation = l.latchedRotation(currentRotation, pov, input);
        assertEquals(Math.PI / 2, desiredRotation.getRadians(), kDelta);
        pov = null;
        desiredRotation = l.latchedRotation(currentRotation, pov, input);
        assertEquals(Math.PI / 2, desiredRotation.getRadians(), kDelta);
    }

    @Test
    void testUnLatch() {
        Experiments.instance.testOverride(Experiment.StickyHeading, false);
        HeadingLatch l = new HeadingLatch();
        Rotation2d currentRotation = new Rotation2d();
        Rotation2d pov = GeometryUtil.kRotation90;
        DriverControl.Velocity input = new DriverControl.Velocity(0, 0, 0);
        Rotation2d desiredRotation = l.latchedRotation(currentRotation, pov, input);
        assertEquals(Math.PI / 2, desiredRotation.getRadians(), kDelta);
        pov = null;
        desiredRotation = l.latchedRotation(currentRotation, pov, input);
        assertEquals(Math.PI / 2, desiredRotation.getRadians(), kDelta);
        input = new DriverControl.Velocity(0, 0, 1);
        desiredRotation = l.latchedRotation(currentRotation, pov, input);
        assertNull(desiredRotation);
    }

    @Test
    void testExplicitUnLatch() {
        Experiments.instance.testOverride(Experiment.StickyHeading, false);
        HeadingLatch l = new HeadingLatch();
        Rotation2d currentRotation = new Rotation2d();
        Rotation2d pov = GeometryUtil.kRotation90;
        DriverControl.Velocity input = new DriverControl.Velocity(0, 0, 0);
        Rotation2d desiredRotation = l.latchedRotation(currentRotation, pov, input);
        assertEquals(Math.PI / 2, desiredRotation.getRadians(), kDelta);
        pov = null;
        desiredRotation = l.latchedRotation(currentRotation, pov, input);
        assertEquals(Math.PI / 2, desiredRotation.getRadians(), kDelta);
        l.unlatch();
        desiredRotation = l.latchedRotation(currentRotation, pov, input);
        assertNull(desiredRotation);
    }

    @Test
    void testSticky() {
        Experiments.instance.testOverride(Experiment.StickyHeading, true);
        HeadingLatch l = new HeadingLatch();
        Rotation2d currentRotation = new Rotation2d(1);
        Rotation2d pov = null;
        // driver steering, latch does nothing.
        DriverControl.Velocity input = new DriverControl.Velocity(0, 0, 1);
        Rotation2d desiredRotation = l.latchedRotation(currentRotation, pov, input);
        assertNull(desiredRotation);

        // let go of the steering stick, latch uses current
        input = new DriverControl.Velocity(0, 0, 0);
        desiredRotation = l.latchedRotation(currentRotation, pov, input);
        assertEquals(1, desiredRotation.getRadians(), kDelta);

        // latch remembers even when current changes
        currentRotation = new Rotation2d(2);
        desiredRotation = l.latchedRotation(currentRotation, pov, input);
        assertEquals(1, desiredRotation.getRadians(), kDelta);
    }
}
