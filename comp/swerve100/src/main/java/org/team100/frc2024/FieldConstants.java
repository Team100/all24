// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.team100.frc2024;

import java.util.EnumMap;
import java.util.Map;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

/** Add your docs here. */
public abstract class FieldConstants {

    private static final Map<Alliance, FieldConstants> fieldConstants = new EnumMap<>(Alliance.class);

    static {
        fieldConstants.put(Alliance.Red, new FieldConstantsRed());
        fieldConstants.put(Alliance.Blue, new FieldConstantsBlue());
    }

    // public double SHOOTER_LEFT_SIDE_Y = 5.158404 ;
    // public double SHOOTER_RIGHT_SIDE_Y = 5.955059 ; //
    // public double SHOOTER_CENTER_Y = 5.596386;
    // public double SHOOTER_CENTER_X = 0.314565;


    // public static double SHOOTER_LEFT_SIDE_Y = 2.095442 ;
    // public static double SHOOTER_RIGHT_SIDE_Y = 3.031812 ; //
    // public static double SHOOTER_CENTER_Y = 2.614550;
    // public static double SHOOTER_CENTER_X = 0.314565;

    public abstract double getShooterLeftSideY();//{
        // return 0;
    // } 

    public abstract double getShooterRightSideY();//{
        // return 0;
    // } 

    public abstract double getShooterCenterY();//{
        // return 0;
    // }

    public abstract double getShooterCenterX();//{
        // return 0;
    // }


    // public static final FieldConstants instance = new FieldConstantsBlue();

    public static FieldConstants get(Alliance alliance){
        return fieldConstants.get(alliance);
    }
    
}


