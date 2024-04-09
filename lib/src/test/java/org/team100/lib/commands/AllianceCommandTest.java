package org.team100.lib.commands;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.junit.jupiter.api.Test;

import edu.wpi.first.hal.AllianceStationID;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import edu.wpi.first.wpilibj2.command.InstantCommand;

class AllianceCommandTest {
    private String output = "none";

        @Test
    void testInvalid() {
        AllianceCommand c = new AllianceCommand(
                new InstantCommand(() -> output = "red"),
                new InstantCommand(() -> output = "blue"));
        DriverStationSim.setAllianceStationId(AllianceStationID.Unknown);
        DriverStationSim.notifyNewData();
        assertTrue(DriverStation.getAlliance().isEmpty());
        assertEquals("none", output);
        c.initialize();
        assertEquals("none", output);
    }

    @Test
    void testRed() {
        AllianceCommand c = new AllianceCommand(
                new InstantCommand(() -> output = "red"),
                new InstantCommand(() -> output = "blue"));
        DriverStationSim.setAllianceStationId(AllianceStationID.Red1);
        DriverStationSim.notifyNewData();
        assertEquals(Alliance.Red, DriverStation.getAlliance().get());
        assertEquals("none", output);
        c.initialize();
        assertEquals("red", output);
    }

    @Test
    void testBlue() {
        AllianceCommand c = new AllianceCommand(
                new InstantCommand(() -> output = "red"),
                new InstantCommand(() -> output = "blue"));
        DriverStationSim.setAllianceStationId(AllianceStationID.Blue1);
        DriverStationSim.notifyNewData();
        assertEquals(Alliance.Blue, DriverStation.getAlliance().get());
        assertEquals("none", output);
        c.initialize();
        assertEquals("blue", output);
    }
}
