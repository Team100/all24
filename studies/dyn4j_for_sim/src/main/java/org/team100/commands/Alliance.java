package org.team100.commands;

import org.team100.robot.RobotSubsystem;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.PrintCommand;

/** Alliance coordinates the behavior of the member robots. */
public class Alliance {

    /** Called by Command.end() */
    public void nextCommand(RobotSubsystem robot, Command command) {
        CommandScheduler.getInstance().schedule(new PrintCommand("blarg"));
    }
}
