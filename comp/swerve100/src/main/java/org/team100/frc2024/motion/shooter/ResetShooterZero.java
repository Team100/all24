package org.team100.frc2024.motion.shooter;

import edu.wpi.first.wpilibj2.command.Command;

public class ResetShooterZero extends Command {
    private final DrumShooter m_shooter;

    public ResetShooterZero(DrumShooter shooter) {
        m_shooter = shooter;
    }

    @Override
    public void initialize() {
        m_shooter.rezero();
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
