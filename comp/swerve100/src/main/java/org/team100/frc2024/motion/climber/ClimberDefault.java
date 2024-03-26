package org.team100.frc2024.motion.climber;

import java.util.function.Supplier;

import org.team100.lib.telemetry.Telemetry;
import org.team100.lib.telemetry.Telemetry.Level;

import edu.wpi.first.wpilibj2.command.Command;

public class ClimberDefault extends Command {
    private static final Telemetry t = Telemetry.get();

    private final ClimberSubsystem m_climber;
    private final Supplier<Double> m_leftSupplier;
    private final Supplier<Double> m_rightSupplier;
    private final Supplier<Boolean> m_overrideSupplier;

    public ClimberDefault(
            ClimberSubsystem climber,
            Supplier<Double> leftSupplier,
            Supplier<Double> rightSupplier,
            Supplier<Boolean> overideSupplier) {
        m_leftSupplier = leftSupplier;
        m_rightSupplier = rightSupplier;
        m_climber = climber;
        m_overrideSupplier = overideSupplier;
        addRequirements(m_climber);
    }

    @Override
    public void initialize() {
        t.log(Level.DEBUG, "ClimberDefault", "command state", "initialize");
    }

    @Override
    public void execute() {
        t.log(Level.DEBUG, "ClimberDefault", "command state", "execute");
        m_climber.setLeft(m_leftSupplier.get());
        m_climber.setRight(m_rightSupplier.get());
    }

    @Override
    public void end(boolean interrupted) {
        t.log(Level.DEBUG, "ClimberDefault", "command state", "end");
    }
}
