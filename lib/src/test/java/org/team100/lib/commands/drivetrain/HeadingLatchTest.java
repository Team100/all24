package org.team100.lib.commands.drivetrain;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertNull;

import org.junit.jupiter.api.Test;
import org.team100.lib.experiments.Experiment;
import org.team100.lib.experiments.Experiments;
import org.team100.lib.geometry.GeometryUtil;
import org.team100.lib.hid.DriverControl;
import org.team100.lib.state.State100;

import edu.wpi.first.math.geometry.Rotation2d;

class HeadingLatchTest {
    private static final double kDelta = 0.001;

    @Test
    void testInit() {
        Experiments.instance.testOverride(Experiment.StickyHeading, false);
        HeadingLatch l = new HeadingLatch();
        State100 s = new State100();
        Rotation2d pov = null;
        DriverControl.Velocity input = new DriverControl.Velocity(0, 0, 0);
        Rotation2d desiredRotation = l.latchedRotation(10, s, pov, input.theta());
        assertNull(desiredRotation);
    }

    @Test
    void testLatch() {
        Experiments.instance.testOverride(Experiment.StickyHeading, false);
        HeadingLatch l = new HeadingLatch();
        State100 s = new State100();
        Rotation2d pov = GeometryUtil.kRotation90;
        DriverControl.Velocity input = new DriverControl.Velocity(0, 0, 0);
        Rotation2d desiredRotation = l.latchedRotation(10, s, pov, input.theta());
        assertEquals(Math.PI / 2, desiredRotation.getRadians(), kDelta);
        pov = null;
        desiredRotation = l.latchedRotation(10, s, pov, input.theta());
        assertEquals(Math.PI / 2, desiredRotation.getRadians(), kDelta);
    }

    @Test
    void testUnLatch() {
        Experiments.instance.testOverride(Experiment.StickyHeading, false);
        HeadingLatch l = new HeadingLatch();
        State100 s = new State100();
        Rotation2d pov = GeometryUtil.kRotation90;
        DriverControl.Velocity input = new DriverControl.Velocity(0, 0, 0);
        Rotation2d desiredRotation = l.latchedRotation(10, s, pov, input.theta());
        assertEquals(Math.PI / 2, desiredRotation.getRadians(), kDelta);
        pov = null;
        desiredRotation = l.latchedRotation(10, s, pov, input.theta());
        assertEquals(Math.PI / 2, desiredRotation.getRadians(), kDelta);
        input = new DriverControl.Velocity(0, 0, 1);
        desiredRotation = l.latchedRotation(10, s, pov, input.theta());
        assertNull(desiredRotation);
    }

    @Test
    void testExplicitUnLatch() {
        Experiments.instance.testOverride(Experiment.StickyHeading, false);
        HeadingLatch l = new HeadingLatch();
        State100 s = new State100();
        Rotation2d pov = GeometryUtil.kRotation90;
        DriverControl.Velocity input = new DriverControl.Velocity(0, 0, 0);
        Rotation2d desiredRotation = l.latchedRotation(10, s, pov, input.theta());
        assertEquals(Math.PI / 2, desiredRotation.getRadians(), kDelta);
        pov = null;
        desiredRotation = l.latchedRotation(10, s, pov, input.theta());
        assertEquals(Math.PI / 2, desiredRotation.getRadians(), kDelta);
        l.unlatch();
        desiredRotation = l.latchedRotation(10, s, pov, input.theta());
        assertNull(desiredRotation);
    }

    @Test
    void testSticky() {
        Experiments.instance.testOverride(Experiment.StickyHeading, true);
        HeadingLatch l = new HeadingLatch();
        State100 s = new State100(1, 1);
        Rotation2d pov = null;
        // driver steering, latch does nothing.
        DriverControl.Velocity input = new DriverControl.Velocity(0, 0, 1);
        Rotation2d desiredRotation = l.latchedRotation(10, s, pov, input.theta());
        assertNull(desiredRotation);

        // let go of the steering stick, latch uses current
        s = new State100(1, 1);
        input = new DriverControl.Velocity(0, 0, 0);
        desiredRotation = l.latchedRotation(10, s, pov, input.theta());
        // max A = 10 rad/s^2
        // V = 1 rad/s
        // t = 0.1 sec
        // dx = 0.05 rad
        // setpoint = 1.05 rad
        assertEquals(1.05, desiredRotation.getRadians(), kDelta);

        // latch remembers even when current changes
        desiredRotation = l.latchedRotation(10, s, pov, input.theta());
        assertEquals(1.05, desiredRotation.getRadians(), kDelta);
    }
}
