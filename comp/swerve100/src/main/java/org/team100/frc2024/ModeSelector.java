// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.team100.frc2024;

import java.util.function.Supplier;

import org.team100.frc2024.RobotState100.State100;

/** Add your docs here. */
public class ModeSelector {

    public static void selectMode(Supplier<Double> pov){
        if(pov.get() != -1){
            if(pov.get() == 90){
                RobotState100.changeRobotState(State100.SHOOTING);
            } else if(pov.get() == 180){
                RobotState100.changeRobotState(State100.AMPING);
            } else if(pov.get() == 270){
                RobotState100.changeRobotState(State100.NONE);
            }
        }
    }
}
