package frc.robot;

import org.team100.robot.RobotContainer;

import edu.wpi.first.hal.ControlWord;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

/**
 * In "test" mode, this mimics the FMS: run auton for 15 sec, then teleop for
 * 2:15, then stop. The right way to do it would be to modify the
 * {@link ControlWord}, but this isn't possible.
 * 
 * The mode is governed by the timer:
 * 
 * time 0-15: auton
 * time 18-153: teleop
 * 
 * exiting test mode early exits whatever mode we're in at the time.
 */
public class Robot extends TimedRobot {
    private enum Mode {
        Start,
        Auton,
        Between,
        Teleop,
        End
    }

    private final RobotContainer m_robotContainer;
    private Timer m_timer;
    private Mode m_mode;

    public Robot() {
        m_robotContainer = new RobotContainer();
        m_timer = new Timer();
        m_mode = Mode.Start;
    }

    @Override
    public void robotPeriodic() {
        CommandScheduler.getInstance().run();
        m_robotContainer.robotPeriodic();
    }

    @Override
    public void robotInit() {
        m_robotContainer.robotInit();
    }

    @Override
    public void teleopInit() {
        // reset each time
        m_robotContainer.teleopInit();
    }

    @Override
    public void teleopExit() {
        m_robotContainer.teleopExit();
    }

    @Override
    public void teleopPeriodic() {
        m_robotContainer.teleopPeriodic();
    }

    @Override
    public void testInit() {
        m_timer.restart();
        m_mode = Mode.Start;
    }

    @Override
    public void testExit() {
        switch (m_mode) {
            case Start:
                break;
            case Auton:
                m_robotContainer.autonomousExit();
                break;
            case Between:
                break;
            case Teleop:
                m_robotContainer.teleopExit();
                break;
            case End:
                break;
        }
    }

    @Override
    public void testPeriodic() {
        switch (m_mode) {
            case Start:
                if (m_timer.get() > 0) {
                    m_mode = Mode.Auton;
                    m_robotContainer.autonomousInit();
                }
                break;
            case Auton:
                if (m_timer.get() < 15) {
                    m_robotContainer.autonomousPeriodic();
                } else {
                    m_mode = Mode.Between;
                    m_robotContainer.autonomousExit();
                }
                break;
            case Between:
                if (m_timer.get() > 18) {
                    m_mode = Mode.Teleop;
                    m_robotContainer.teleopInit();
                }
                break;
            case Teleop:
                if (m_timer.get() < 153) {
                    m_robotContainer.teleopPeriodic();
                } else {
                    m_mode = Mode.End;
                    m_robotContainer.teleopExit();
                }
                break;
            case End:
                //
                break;
        }

    }

    @Override
    public void autonomousInit() {
        m_robotContainer.autonomousInit();
    }

    @Override
    public void autonomousExit() {
        m_robotContainer.autonomousExit();
    }

    @Override
    public void autonomousPeriodic() {
        m_robotContainer.autonomousPeriodic();
    }

}
