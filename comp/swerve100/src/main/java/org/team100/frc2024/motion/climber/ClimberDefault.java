package org.team100.frc2024.motion.climber;

import java.util.function.DoubleSupplier;

import org.team100.lib.dashboard.Glassy;
import org.team100.lib.logging.Level;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.logging.LoggerFactory.DoubleLogger;
import org.team100.lib.motion.mechanism.LinearMechanism;

import edu.wpi.first.wpilibj2.command.Command;

public class ClimberDefault extends Command implements Glassy {
    private final ClimberSubsystem m_climber;
    private final DoubleSupplier m_left;
    private final DoubleSupplier m_right;

    // LOGGERS
    private final DoubleLogger m_log_left_manual;
    private final DoubleLogger m_log_right_manual;

    public ClimberDefault(
            LoggerFactory logger,
            ClimberSubsystem climber,
            DoubleSupplier leftSupplier,
            DoubleSupplier rightSupplier) {
        LoggerFactory child = logger.child(this);
        m_log_left_manual = child.doubleLogger(Level.TRACE, "left manual");
        m_log_right_manual = child.doubleLogger(Level.TRACE, "right manual");
        m_climber = climber;
        m_left = leftSupplier;
        m_right = rightSupplier;
        addRequirements(m_climber);
    }

    @Override
    public void initialize() {
        m_climber.setClimbingForce();
    }

    @Override
    public void execute() {
        manual(m_log_left_manual, m_left, m_climber.getLeft());
        manual(m_log_right_manual, m_right, m_climber.getRight());
    }

    private void manual(
            DoubleLogger log,
            DoubleSupplier inputSupplier,
            LinearMechanism mech) {
        double input = inputSupplier.getAsDouble();
        log.log(() -> input);
        mech.setDutyCycle(input);
    }
}
