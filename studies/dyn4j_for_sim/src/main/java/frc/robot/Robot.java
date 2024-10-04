package frc.robot;

import org.team100.field.Score;
import org.team100.field.ScoreDisplay;
import org.team100.field.SimulatedFMS;
import org.team100.lib.async.Async;
import org.team100.lib.async.AsyncFactory;
import org.team100.lib.framework.TimedRobot100;
import org.team100.lib.logging.Level;
import org.team100.lib.logging.LevelPoller;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.logging.Logging;
import org.team100.lib.util.Util;
import org.team100.robot.RobotContainer;
import org.team100.sim.SimWorld;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

public class Robot extends TimedRobot100 {
    private final RobotContainer m_robotContainer;
    private final SimulatedFMS m_fms;
    private final ScoreDisplay m_display;

    public Robot() {
        final AsyncFactory asyncFactory = new AsyncFactory(this);
        final Async async = asyncFactory.get();

        final Logging logging = Logging.instance();
        final LevelPoller poller = new LevelPoller(async, logging::setLevel, Level.COMP);
        Util.printf("Using log level %s\n", poller.getLevel().name());
        final LoggerFactory fieldLogger = logging.fieldLogger;

        Score blueScore = new Score();
        Score redScore = new Score();
        SimWorld world = new SimWorld(fieldLogger, blueScore, redScore);
        m_robotContainer = new RobotContainer(fieldLogger, world);
        m_fms = new SimulatedFMS();
        m_display = new ScoreDisplay(
                world.getScorekeeper(),
                blueScore,
                redScore,
                m_fms);
    }

    @Override
    public void robotPeriodic() {
        CommandScheduler.getInstance().run();
        m_robotContainer.robotPeriodic();
        NetworkTableInstance.getDefault().flush();
    }

    @Override
    public void robotInit() {
        m_robotContainer.robotInit();
        m_fms.start();
        m_display.start();
    }

    @Override
    public void teleopInit() {
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
