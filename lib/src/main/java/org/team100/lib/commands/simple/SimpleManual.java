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
    public static class Config {
        public double maxDutyCycle = 0.5;
        public double maxVelocity = 1;
        public double maxPosition = 1;
    }

    private final Config m_config = new Config();
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
                m_simple.setDutyCycle(m_config.maxDutyCycle * m_input.getAsDouble());
                break;
            case VELOCITY:
                m_simple.setVelocity(m_config.maxDutyCycle * m_input.getAsDouble());
                break;
            case POSITION:
                m_simple.setPosition(m_config.maxPosition * m_input.getAsDouble());
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
