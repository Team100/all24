package org.team100.lib.commands.simple;

import java.util.function.DoubleSupplier;

import org.team100.lib.motion.simple.SimpleSubsystem;

import edu.wpi.first.wpilibj2.command.Command;

public class SimpleManual extends Command {
    public static class Config {
        public double maxDutyCycle = 0.5;
    }

    private final Config m_config = new Config();
    private final SimpleSubsystem m_simple;
    private final DoubleSupplier m_v;

    public SimpleManual(SimpleSubsystem simple, DoubleSupplier v) {
        m_simple = simple;
        m_v = v;
        addRequirements(simple);
    }

    @Override
    public void execute() {
        m_simple.set(m_config.maxDutyCycle * m_v.getAsDouble());
    }

    @Override
    public void end(boolean interrupted) {
        m_simple.set(0);
    }
}
