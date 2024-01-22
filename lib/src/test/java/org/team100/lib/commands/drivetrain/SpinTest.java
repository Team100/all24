package org.team100.lib.commands.drivetrain;

import static org.junit.jupiter.api.Assertions.assertEquals;

import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.Test;
import org.team100.lib.experiments.Experiment;
import org.team100.lib.experiments.Experiments;
import org.team100.lib.motion.drivetrain.Fixture;
import org.team100.lib.testing.TimelessTest;

class SpinTest extends TimelessTest {
    private static final double kDelta = 0.01;

    Fixture fixture = new Fixture();

    @AfterEach
    void close() {
        fixture.close();
    }

    @Test
    void testSimple() {
        Experiments.instance.testOverride(Experiment.UseSetpointGenerator, true);

        Spin command = new Spin(fixture.drive, fixture.controller);
        command.initialize();
        stepTime(0.02);
        // starts from rest
        command.execute();
        assertEquals(0, command.m_center.getX(), kDelta);
        assertEquals(0, command.m_center.getY(), kDelta);
        assertEquals(0, command.m_initialRotation, kDelta);
        assertEquals(0, command.m_angleRad, kDelta);
        assertEquals(0.01, command.m_speedRad_S, kDelta);
        assertEquals(0, fixture.drive.desiredStates()[0].speedMetersPerSecond, kDelta);
        assertEquals(-0.785, fixture.drive.desiredStates()[0].angle.getRadians(), 0.01);

        stepTime(5);
        command.execute();
        assertEquals(0, command.m_center.getX(), kDelta);
        assertEquals(0, command.m_center.getY(), kDelta);
        assertEquals(0, command.m_initialRotation, kDelta);
        assertEquals(2.5, command.m_angleRad, kDelta);
        assertEquals(0.5, command.m_speedRad_S, kDelta);
        // test drivetrain is 0.5m on a side, thus radius
        // of about 0.353 m. so 0.5 rad/s

        assertEquals(-0.176, fixture.drive.desiredStates()[0].speedMetersPerSecond, kDelta);
        assertEquals(0.176, fixture.drive.desiredStates()[1].speedMetersPerSecond, kDelta);
        assertEquals(Math.PI / 4, fixture.drive.desiredStates()[1].angle.getRadians(), kDelta);
        assertEquals(-0.176, fixture.drive.desiredStates()[2].speedMetersPerSecond, kDelta);
        assertEquals(Math.PI / 4, fixture.drive.desiredStates()[2].angle.getRadians(), kDelta);
        assertEquals(0.176, fixture.drive.desiredStates()[3].speedMetersPerSecond, kDelta);
        assertEquals(-Math.PI / 4, fixture.drive.desiredStates()[3].angle.getRadians(), kDelta);
    }

}
