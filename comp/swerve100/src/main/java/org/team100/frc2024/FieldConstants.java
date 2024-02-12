// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.team100.frc2024;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

/** Add your docs here. */
public class FieldConstants {
    public double SHOOTER_LEFT_SIDE_Y = 0 ;
    public double SHOOTER_RIGHT_SIDE_Y = 0; //
    public double SHOOTER_CENTER_Y = 0;
    public double SHOOTER_CENTER_X = 0;

    public static final FieldConstants instance = get();

    private static FieldConstants get(){
        if(DriverStation.getAlliance().get() == Alliance.Blue){
            return new FieldConstantsBlue();
        }

        return new FieldConstantsRed();
    }

    
}


