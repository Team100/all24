package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

public class Robot extends TimedRobot {
    private final RobotContainer m_robotContainer;

    public Robot() {
        m_robotContainer = new RobotContainer();
    }

    @Override
    public void robotInit() {
        // nothing
    }

    @Override
    public void simulationPeriodic() {
        // Here we calculate the battery voltage based on drawn current.
        // As our robot draws more power from the battery its voltage drops.
        // The estimated voltage is highly dependent on the battery's internal
        // resistance.
        double drawCurrent = m_robotContainer.getRobotDrive().getDrawnCurrentAmps();
        double loadedVoltage = BatterySim.calculateDefaultBatteryLoadedVoltage(drawCurrent);
        RoboRioSim.setVInVoltage(loadedVoltage);
    }

    @Override
    public void robotPeriodic() {
        CommandScheduler.getInstance().run();
    }

    @Override
    public void autonomousInit() {
        m_robotContainer.getAutonomousCommand().schedule();
    }

    @Override
    public void disabledInit() {
        CommandScheduler.getInstance().cancelAll();
        m_robotContainer.zeroAllOutputs();
    }
}
