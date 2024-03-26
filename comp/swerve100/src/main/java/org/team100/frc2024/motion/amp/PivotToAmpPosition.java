package org.team100.frc2024.motion.amp;

import edu.wpi.first.wpilibj2.command.Command;

public class PivotToAmpPosition extends Command {
    private final AmpSubsystem m_amp;

    public PivotToAmpPosition(AmpSubsystem amp) {
        m_amp = amp;
        addRequirements(amp);
    }

    @Override
    public void execute() {
        m_amp.setAmpPosition(2.066);
    }
}
