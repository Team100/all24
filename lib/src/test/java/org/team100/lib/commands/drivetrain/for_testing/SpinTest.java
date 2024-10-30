package org.team100.lib.commands.drivetrain.for_testing;

import static org.junit.jupiter.api.Assertions.assertEquals;

import org.junit.jupiter.api.Test;
import org.team100.lib.experiments.Experiment;
import org.team100.lib.experiments.Experiments;
import org.team100.lib.motion.drivetrain.Fixtured;
import org.team100.lib.testing.Timeless;

class SpinTest extends Fixtured implements Timeless {
    private static final double kDelta = 0.01;

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
        assertEquals(0, fixture.drive.getSwerveLocal().getDesiredStates()[0].speedMetersPerSecond, kDelta);
        // larger tolerance makes this not move?  not sure.
        assertEquals(0, fixture.drive.getSwerveLocal().getDesiredStates()[0].angle.get().getRadians(), 0.01);

        for (int i = 0; i < 273; ++i) {
            command.execute();
        }
        assertEquals(0, command.m_center.getX(), kDelta);
        assertEquals(0, command.m_center.getY(), kDelta);
        assertEquals(0, command.m_initialRotation, kDelta);
        assertEquals(2.5, command.m_angleRad, kDelta);
        assertEquals(0.5, command.m_speedRad_S, kDelta);
        // test drivetrain is 0.5m on a side, thus radius
        // of about 0.353 m. so 0.5 rad/s

        assertEquals(-0.176, fixture.drive.getSwerveLocal().getDesiredStates()[0].speedMetersPerSecond, kDelta);
        assertEquals(0.176, fixture.drive.getSwerveLocal().getDesiredStates()[1].speedMetersPerSecond, kDelta);
        assertEquals(Math.PI / 4, fixture.drive.getSwerveLocal().getDesiredStates()[1].angle.get().getRadians(), kDelta);
        assertEquals(-0.176, fixture.drive.getSwerveLocal().getDesiredStates()[2].speedMetersPerSecond, kDelta);
        assertEquals(Math.PI / 4, fixture.drive.getSwerveLocal().getDesiredStates()[2].angle.get().getRadians(), kDelta);
        assertEquals(0.176, fixture.drive.getSwerveLocal().getDesiredStates()[3].speedMetersPerSecond, kDelta);
        assertEquals(-Math.PI / 4, fixture.drive.getSwerveLocal().getDesiredStates()[3].angle.get().getRadians(), kDelta);
    }

}
