package frc.robot;

import org.team100.field.Score;
import org.team100.field.ScoreDisplay;
import org.team100.field.SimulatedFMS;
import org.team100.lib.telemetry.FieldLogger;
import org.team100.lib.telemetry.Telemetry;
import org.team100.robot.RobotContainer;
import org.team100.sim.SimWorld;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

public class Robot extends TimedRobot {
    private final RobotContainer m_robotContainer;
    private final SimulatedFMS m_fms;
    private final ScoreDisplay m_display;

    public Robot() {
        final Telemetry telemetry = Telemetry.get();
        final FieldLogger logger = telemetry.fieldLogger(true, false);

        Score blueScore = new Score();
        Score redScore = new Score();
        SimWorld world = new SimWorld(logger, blueScore, redScore);
        m_robotContainer = new RobotContainer(logger, world);
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
