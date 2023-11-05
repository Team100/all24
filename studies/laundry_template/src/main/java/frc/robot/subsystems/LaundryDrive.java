package frc.robot.subsystems;

public interface LaundryDrive {

    void enable();

    void disable();

    void autonomousInit();

    void autonomousPeriodic();

    void teleopPeriodic();

}