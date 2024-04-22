package frc.robot;

import org.team100.robot.RobotContainer;


import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

public class Robot extends TimedRobot {

    private final RobotContainer m_robotContainer;

    public Robot() {
        m_robotContainer = new RobotContainer();
    }

    @Override
    public void robotPeriodic() {
        CommandScheduler.getInstance().run();
    }

    @Override
    public void teleopInit() {
        m_robotContainer.init();
    }

    @Override
    public void teleopPeriodic() {
        m_robotContainer.periodic();
    }
}
