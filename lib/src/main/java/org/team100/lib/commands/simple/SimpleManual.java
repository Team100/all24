package org.team100.lib.commands.simple;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import org.team100.lib.motion.simple.SimpleSubsystem;

import edu.wpi.first.wpilibj2.command.Command;

/**
 * Manual control of a single degree of freedom.
 * 
 * Three modes are provided: duty-cycle, closed-loop velocity, and closed-loop
 * position.
 */
public class SimpleManual extends Command {
    private static final double kMaxDutyCycle = 0.5;
    private static final double kMaxVelocity = 2; // m/s
    private static final double kMaxPosition = 2; // m

    private final Supplier<SimpleManualMode.Mode> m_mode;
    private final SimpleSubsystem m_simple;
    private final DoubleSupplier m_input;

    public SimpleManual(
            Supplier<SimpleManualMode.Mode> mode,
            SimpleSubsystem simple,
            DoubleSupplier input) {
        m_mode = mode;
        m_simple = simple;
        m_input = input;
        addRequirements(simple);
    }

    @Override
    public void execute() {
        SimpleManualMode.Mode manualMode = m_mode.get();
        if (manualMode == null) {
            return;
        }
        switch (manualMode) {
            case DUTY_CYCLE:
                m_simple.setDutyCycle(kMaxDutyCycle * m_input.getAsDouble());
                break;
            case VELOCITY:
                m_simple.setVelocity(kMaxVelocity * m_input.getAsDouble());
                break;
            case POSITION:
                m_simple.setPosition(kMaxPosition * m_input.getAsDouble());
                break;
            default:
                System.out.println("invalid manual mode: " + manualMode.name());
                break;
        }
    }

    @Override
    public void end(boolean interrupted) {
        m_simple.setDutyCycle(0);
    }
}
