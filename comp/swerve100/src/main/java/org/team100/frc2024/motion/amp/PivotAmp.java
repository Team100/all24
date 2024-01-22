package org.team100.frc2024.motion.amp;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;

public class PivotAmp extends Command {
    private final AmpSubsystem m_amp;
    private final Supplier<Double> m_ampVelocity;

    public PivotAmp(AmpSubsystem amp, Supplier<Double> ampVelocity) {
        m_amp = amp;
        m_ampVelocity = ampVelocity;
        addRequirements(m_amp);
    }

    @Override
    public void execute() {
        // for now, don't do anything.
        m_amp.stop();

        // if (m_ampVelocity.get() == 0) {
        //     m_amp.setAmpVelocity(m_ampVelocity.get());
        // } else {
        //     m_amp.setAmpPosition(m_amp.getLeftAmpPosition(), m_amp.getRightAmpPosition());
        // }
    }
}
