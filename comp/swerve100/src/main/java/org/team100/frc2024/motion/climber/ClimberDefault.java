package org.team100.frc2024.motion.climber;

import java.util.function.DoubleSupplier;

import org.team100.lib.dashboard.Glassy;
import org.team100.lib.motion.LinearMechanismInterface;
import org.team100.lib.telemetry.SupplierLogger;
import org.team100.lib.telemetry.Telemetry.Level;

import edu.wpi.first.wpilibj2.command.Command;

public class ClimberDefault extends Command implements Glassy {
    private final SupplierLogger m_logger;
    private final ClimberSubsystem m_climber;
    private final DoubleSupplier m_left;
    private final DoubleSupplier m_right;

    public ClimberDefault(
            SupplierLogger logger,
            ClimberSubsystem climber,
            DoubleSupplier leftSupplier,
            DoubleSupplier rightSupplier) {
        m_logger = logger.child(this);
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
        manual("left", m_left, m_climber.getLeft());
        manual("right", m_right, m_climber.getRight());

    }

    private void manual(
            String name,
            DoubleSupplier inputSupplier,
            LinearMechanismInterface mech) {
        double input = inputSupplier.getAsDouble();
        m_logger.logDouble(Level.TRACE, name + "manual", () -> input);
        mech.setDutyCycle(input);
    }

    @Override
    public String getGlassName() {
        return "ClimberDefault";
    }
}
