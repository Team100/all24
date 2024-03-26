package org.team100.frc2024.motion;

import java.util.ArrayList;

import edu.wpi.first.wpilibj2.command.Command;

public class CommandGroup {

    ArrayList<Command> commandList = new ArrayList<>();

    public CommandGroup(Command... commands) {
        for (Command command : commands) {
            commandList.add(command);
        }
    }

    public Command get(int i) {
        return commandList.get(i);
    }
}
