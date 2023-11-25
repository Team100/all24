package org.team100.lib.commands.simple;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import org.team100.lib.motion.simple.SimpleSubsystem;

import edu.wpi.first.wpilibj2.command.Command;

public class SimpleManual extends Command {
    public static class Config {
        public double maxDutyCycle = 0.5;
    }

    private final Config m_config = new Config();
    private final Supplier<SimpleManualMode.Mode> m_mode;
    private final SimpleSubsystem m_simple;
    private final DoubleSupplier m_v;

    public SimpleManual(
            Supplier<SimpleManualMode.Mode> mode,
            SimpleSubsystem simple,
            DoubleSupplier v) {
        m_mode = mode;
        m_simple = simple;
        m_v = v;
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
                m_simple.setDutyCycle(m_config.maxDutyCycle * m_v.getAsDouble());
                break;
            case VELOCITY:
                break;
            default:
                // do nothing
                break;
        }

    }

    @Override
    public void end(boolean interrupted) {
        m_simple.setDutyCycle(0);
    }
}
