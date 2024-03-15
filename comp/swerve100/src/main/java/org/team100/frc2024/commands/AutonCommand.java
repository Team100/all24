package org.team100.frc2024.commands;

import java.util.Map;
import java.util.function.Supplier;

import org.team100.frc2024.config.AutonChooser;
import org.team100.frc2024.config.AutonChooser.Routine;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SelectCommand;

public class AutonCommand extends SelectCommand<AutonChooser.Routine> {

    public AutonCommand(
            Map<Routine, Command> commands,
            Supplier<Routine> selector) {
        super(commands, selector);
    }
}
