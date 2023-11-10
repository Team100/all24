package org.team100.lib.commands;

import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.Subsystem;

/**
 * Runs once and never ends. This is useful to activate a subsystem
 * while a button is held, reverting to the default behavior when the button is
 * released, cancelling this command and starting the default command. It's
 * identical to InstantCommand with "isFinished" returning false.
 */
public class InitCommand extends FunctionalCommand {
    /** Runs once and never ends. */
    public InitCommand(Runnable toRun, Subsystem... requirements) {
        super(
                toRun,
                () -> {
                },
                interrupted -> {
                },
                () -> false,
                requirements);
    }

}
