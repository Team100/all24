package org.team100.frc2024;

import org.team100.frc2024.motion.intake.Intake;

public class RobotState100 {

    public enum State100{
        MANUAL, SHOOTING, AMPING, NONE
    }

    public enum ShooterState100{
        FEED, NONE, DEFAULTSHOOT, READYTOSHOOT, DOWN, STOP
    }

    public enum AmpState100{
        UP, DOWN, FEED, OUTTAKE, NONE
    }

    public enum IntakeState100{
        INTAKE, OUTTAKE, STOP, NONE
    }

     public enum FeederState100{
        FEED, STOP
    }

    public static State100 currentRobotState = State100.SHOOTING;
    public static ShooterState100 currentShooterState = ShooterState100.STOP;
    public static AmpState100 currentAmpState = AmpState100.NONE;
    public static IntakeState100 currentIntakeState = IntakeState100.STOP;
    public static FeederState100 currentFeederState = FeederState100.STOP;


    public static void changeRobotState(State100 state){
        currentRobotState = state;
    }

    public static State100 getRobotState(){
        return currentRobotState;
    }

    public static void changeShooterState(ShooterState100 state){
        currentShooterState = state;
    }

    public static ShooterState100 getShooterState(){
        return currentShooterState;
    }

    public static void changeAmpState(AmpState100 state){
        currentAmpState = state;
    }

    public static AmpState100 getAmpState(){
        return currentAmpState;
    }

    public static void changeIntakeState(IntakeState100 state){
        currentIntakeState = state;
    }

    public static IntakeState100 getIntakeState(){
        return currentIntakeState;
    }

    public static void changeFeederState(FeederState100 state){
        currentFeederState = state;
    }

    public static FeederState100 getFeederState(){
        return currentFeederState;
    }
    
    
}

