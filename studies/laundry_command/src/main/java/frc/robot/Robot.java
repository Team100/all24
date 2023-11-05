package frc.robot;

import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.util.WPILibVersion;
import edu.wpi.first.wpilibj2.command.CommandScheduler;


public class Robot extends TimedRobot {
    private RobotContainer m_robotContainer;

    public Robot() {
        m_robotContainer = new RobotContainer();
    }

    @Override
    public void robotInit() {
        System.out.printf("WPILib Version: %s\n", WPILibVersion.Version); // 2023.2.1
        System.out.printf("RoboRIO serial number: %s\n", RobotController.getSerialNumber());
        DataLogManager.start();
    }

    @Override
    public void robotPeriodic() {
        CommandScheduler.getInstance().run();
    }

    @Override
    public void autonomousInit() {
        m_robotContainer.scheduleAuton();
    }

    @Override
    public void teleopInit() {
        m_robotContainer.cancelAuton();
    }
}
