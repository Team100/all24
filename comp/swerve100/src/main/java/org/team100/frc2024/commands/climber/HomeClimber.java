package org.team100.frc2024.commands.climber;

import org.team100.frc2024.motion.climber.ClimberSubsystem;
import org.team100.lib.telemetry.SupplierLogger;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;

/**
 * Set the current limit (torque) to something above friction level but below
 * damage.
 * Run the mechanism at low effort, watching the actual speed.
 * When it stops, zero it.
 */
public class HomeClimber extends Command {
    private static final double kTargetSpeedM_S = 0.02;
    private static final double kThresholdSpeedM_S = 0.002;
    private static final double kHomeTimeS = 0.5;
    private final SupplierLogger m_logger;
    private final ClimberSubsystem m_climber;
    private final Timer m_timer;

    public HomeClimber(
            SupplierLogger logger,
            ClimberSubsystem climber) {
        m_logger = logger;
        m_climber = climber;
        m_timer = new Timer();
    }

    @Override
    public void initialize() {
        m_timer.restart();
    }

    @Override
    public void execute() {
        m_climber.setLeft(kTargetSpeedM_S);
        m_climber.setRight(kTargetSpeedM_S);
    }

    @Override
    public boolean isFinished() {
        return m_timer.hasElapsed(kHomeTimeS) && m_climber.getVelocityM_S() < kThresholdSpeedM_S;
    }

    @Override
    public void end(boolean interrupted) {
        if (!interrupted)
            m_climber.zeroClimbers();
    }

}
