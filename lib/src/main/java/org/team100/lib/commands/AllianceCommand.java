package org.team100.lib.commands;

import java.util.Map;
import java.util.Optional;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SelectCommand;

/**
 * Executes the red or blue command based on the current alliance.
 */
public class AllianceCommand extends SelectCommand<AllianceCommand.Select> {
    /**
     * We can't use null as a selector output so make an explicit unknown.
     */
    public enum Select {
        Red,
        Blue,
        Unknown
    }

    public AllianceCommand(Command red, Command blue) {
        // Leaving unknown out of the map means we get the default PrintCommand.
        super(Map.of(Select.Red, red, Select.Blue, blue, Select.Unknown, err()),
                AllianceCommand::selector);
    }

    private static Select selector() {
        Optional<Alliance> opt = DriverStation.getAlliance();
        if (opt.isEmpty())
            return Select.Unknown;
        switch (opt.get()) {
            case Red:
                return Select.Red;
            case Blue:
                return Select.Blue;
            default:
                return Select.Unknown;
        }
    }

    private static Command err() {
        // each instance gets its own error printer to avoid tripping
        // the multi-composition detector
        return new PrintCommand("AllianceCommand: no alliance!");
    }
}
