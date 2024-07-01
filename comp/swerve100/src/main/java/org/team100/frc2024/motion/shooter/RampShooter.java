package org.team100.frc2024.motion.shooter;

import edu.wpi.first.wpilibj2.command.Command;

public class RampShooter extends Command {
    private final DrumShooter m_shooter;

    public RampShooter(DrumShooter shooter) {
        m_shooter = shooter;
        addRequirements(shooter);
    }

    @Override
    public void execute() {
        m_shooter.forward();
    }
}
