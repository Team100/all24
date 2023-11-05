package org.team100.frc2023.commands.arm;

import org.team100.frc2023.subsystems.arm.ArmInterface;
import org.team100.lib.indicator.LEDIndicator;
import org.team100.lib.indicator.LEDIndicator.State;

import edu.wpi.first.wpilibj2.command.Command;

public class SetCubeMode extends Command {
    private final ArmInterface m_arm;
    private final LEDIndicator m_indicator;

    public SetCubeMode(ArmInterface arm, LEDIndicator indicator) {
        m_arm = arm;
        m_indicator = indicator;
        addRequirements(m_arm.subsystem());
    }

    @Override
    public void initialize() {
        m_arm.setCubeMode(true);
        m_indicator.set(State.PURPLE);
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
