package frc.robot;

import org.team100.persistent_parameter.RobotContainer;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

/** This just shows that the sim UI and the persistent file both work. */
public class Robot extends TimedRobot {
    private final RobotContainer m_robotContainer;

    public Robot() {
        m_robotContainer = new RobotContainer();
    }

    @Override
    public void robotPeriodic() {
        m_robotContainer.doNothing();
        CommandScheduler.getInstance().run();
    }
}
