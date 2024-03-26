package org.team100.frc2024.motion;

import org.team100.frc2024.motion.amp.AmpSubsystem;

import edu.wpi.first.wpilibj2.command.Command;

public class StowAmpCommand extends Command {
    private final AmpSubsystem m_amp;
    private boolean finished = false;

    public StowAmpCommand(AmpSubsystem amp) {
        m_amp = amp;
        addRequirements(m_amp);
    }

    @Override
    public void initialize() {
        m_amp.reset();
        m_amp.setAmpPosition(0.064230);
    }

    @Override
    public void execute() {
        m_amp.setAmpPosition(0.064230);

        if (Math.abs(m_amp.getPositionRad() - 0.064230) < 0.05) {
            finished = true;
        }
    }

    @Override
    public void end(boolean interrupted) {
        finished = false;
    }

    @Override
    public boolean isFinished() {
        return finished;
    }
}
