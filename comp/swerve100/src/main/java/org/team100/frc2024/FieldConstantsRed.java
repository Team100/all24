package org.team100.frc2024;

public class FieldConstantsRed extends FieldConstants {
    public double SHOOTER_LEFT_SIDE_Y = 2.095442 ;
    public double SHOOTER_RIGHT_SIDE_Y = 3.031812 ; //
    public double SHOOTER_CENTER_Y = 2.614550;
    public double SHOOTER_CENTER_X = 0.314565;

    @Override
    public double getShooterLeftSideY(){
        return SHOOTER_LEFT_SIDE_Y;
    } 

    @Override
    public double getShooterRightSideY(){
        return SHOOTER_RIGHT_SIDE_Y;
    } 

    public double getShooterCenterY(){
        return SHOOTER_CENTER_Y;
    }

    public double getShooterCenterX(){
        return SHOOTER_CENTER_X;
    }
}
