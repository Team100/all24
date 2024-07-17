package org.team100.frc2024.motion.climber;

import java.util.OptionalDouble;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleConsumer;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import org.team100.lib.controller.State100;
import org.team100.lib.dashboard.Glassy;
import org.team100.lib.profile.TrapezoidProfile100;
import org.team100.lib.telemetry.SupplierLogger;
import org.team100.lib.telemetry.Telemetry.Level;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;

public class ClimberDefault extends Command implements Glassy {
    private static final double kMaxPositionM = 0.3;
    private static final double kUpPositionM = 0.28;
    private static final double kDownPosition = 0.02;
    private static final double kMinPositionM = 0.01;

    private final SupplierLogger m_logger;
    private final ClimberSubsystem m_climber;
    private final DoubleSupplier m_leftSupplier;
    private final DoubleSupplier m_rightSupplier;

    public ClimberDefault(
            SupplierLogger logger,
            ClimberSubsystem climber,
            DoubleSupplier leftSupplier,
            DoubleSupplier rightSupplier) {
        m_logger = logger.child(this);
        m_climber = climber;
        m_leftSupplier = leftSupplier;
        m_rightSupplier = rightSupplier;
        addRequirements(m_climber);
    }

    @Override
    public void initialize() {
        m_climber.setClimbingForce();
    }

    @Override
    public void execute() {
        manual("left",
                m_climber::getLeftPositionM,
                m_leftSupplier,
                m_climber::setLeft);
        manual("right",
                m_climber::getRightPositionM,
                m_rightSupplier,
                m_climber::setRight);

    }

    private void manual(
            String name,
            Supplier<OptionalDouble> position,
            DoubleSupplier inputSupplier,
            DoubleConsumer setter) {
        OptionalDouble posOpt = position.get();
        if (posOpt.isEmpty())
            return;
        double positionM = posOpt.getAsDouble();
        double input = inputSupplier.getAsDouble();
        m_logger.logDouble(Level.TRACE, name + "manual", () -> input);
        if ((input >= 0 && positionM <= kMaxPositionM)
                || (input <= 0 && positionM >= kMinPositionM)) {
            setter.accept(input);
        } else {
            setter.accept(0);
        }
    }

    @Override
    public String getGlassName() {
        return "ClimberDefault";
    }
}
