package org.team100.frc2024.motion.shooter;

import org.team100.frc2024.RobotState100;
import org.team100.frc2024.RobotState100.ShooterState100;

import edu.wpi.first.wpilibj2.command.Command;

public class SetDefaultShoot extends Command {
    private final ShooterState100 m_state;
    private final Shooter m_shooter;

    public SetDefaultShoot(Shooter shooter, ShooterState100 state) {
        m_state = state;
        m_shooter = shooter;
    }

    @Override
    public void initialize() {
        m_shooter.reset();
        RobotState100.changeShooterState(m_state);
    }

    @Override
    public void end(boolean interrupted) {
        RobotState100.changeShooterState(ShooterState100.STOP);
    }
}
