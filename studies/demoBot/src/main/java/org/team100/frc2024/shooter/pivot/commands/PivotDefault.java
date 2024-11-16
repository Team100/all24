package org.team100.frc2024.shooter.pivot.commands;

import java.util.function.Supplier;

import org.team100.frc2024.shooter.pivot.PivotSubsystem;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;

public class PivotDefault extends Command {

    private final Supplier<Double> m_twistSupplier;
    private final PivotSubsystem m_pivot;

    private final double rate = -0.5;

    public PivotDefault(Supplier<Double> twistSupplier, PivotSubsystem pivot) {
        m_twistSupplier = twistSupplier;
        m_pivot = pivot;
        addRequirements(m_pivot);
    }

    @Override
    public void initialize() {}

    @Override
    public void execute() {
        m_pivot.dutyCycle(MathUtil.applyDeadband(m_twistSupplier.get(), 0.05)/2);
    }

    @Override
    public void end(boolean interrupted) {
        m_pivot.stop();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
