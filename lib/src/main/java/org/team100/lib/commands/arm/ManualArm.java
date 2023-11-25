package org.team100.lib.commands.arm;

import java.util.function.DoubleSupplier;

import org.team100.lib.motion.arm.ArmSubsystem;

import edu.wpi.first.wpilibj2.command.Command;

/**
 * Direct manual arm control in joint coordinates.
 * 
 * There are no safety features like soft limits or current limits or anything,
 * this is the simplest possible direct manual control.
 */
public class ManualArm extends Command {
    public static class Config {
        public double maxDutyCycle = 0.5;
    }

    private final Config m_config = new Config();
    private final ArmSubsystem m_arm;
    private final DoubleSupplier m_v1;
    private final DoubleSupplier m_v2;

    public ManualArm(ArmSubsystem arm, DoubleSupplier v1, DoubleSupplier v2) {
        m_arm = arm;
        m_v1 = v1;
        m_v2 = v2;
        addRequirements(arm);
    }

    @Override
    public void execute() {
        m_arm.set(
                m_config.maxDutyCycle * m_v1.getAsDouble(),
                m_config.maxDutyCycle * m_v2.getAsDouble());
    }

    @Override
    public void end(boolean interrupted) {
        m_arm.set(0, 0);
    }
}
