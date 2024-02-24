// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.team100.frc2024;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

/** Add your docs here. */
public class FieldConstants {
    // public double SHOOTER_LEFT_SIDE_Y = 5.158404 ;
    // public double SHOOTER_RIGHT_SIDE_Y = 5.955059 ; //
    // public double SHOOTER_CENTER_Y = 5.596386;
    // public double SHOOTER_CENTER_X = 0.314565;


    // public static double SHOOTER_LEFT_SIDE_Y = 2.095442 ;
    // public static double SHOOTER_RIGHT_SIDE_Y = 3.031812 ; //
    // public static double SHOOTER_CENTER_Y = 2.614550;
    // public static double SHOOTER_CENTER_X = 0.314565;

    public double getShooterLeftSideY(){
        return 0;
    } 

    public double getShooterRightSideY(){
        return 0;
    } 

    public double getShooterCenterY(){
        return 0;
    }

    public double getShooterCenterX(){
        return 0;
    }


    public static final FieldConstants instance = new FieldConstantsBlue();

    private static FieldConstants get(){
        if(DriverStation.getAlliance().get() == Alliance.Blue){
            return new FieldConstantsBlue();
        }

        return new FieldConstantsRed();
    }

    
}


