// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.team100.frc2024.motion;

import java.util.ArrayList;

import edu.wpi.first.wpilibj2.command.Command;

/** Add your docs here. */
public class CommandGroup {

    ArrayList<Command> commandList = new ArrayList<>();

    public CommandGroup(Command ... commands){
        for(Command command : commands){
            commandList.add(command);
        }
    }

    public Command get(int i){
        return commandList.get(i);
    }
}
