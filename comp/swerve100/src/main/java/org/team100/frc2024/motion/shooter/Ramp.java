package org.team100.frc2024.motion.shooter;

import org.team100.frc2024.RobotState100;
import org.team100.frc2024.RobotState100.ShooterState100;

import edu.wpi.first.wpilibj2.command.Command;

public class Ramp extends Command {

    @Override
    public void initialize() {
        RobotState100.changeShooterState(ShooterState100.DEFAULTSHOOT);
    }

    @Override
    public void end(boolean interrupted) {
        RobotState100.changeShooterState(ShooterState100.STOP);
    }

}
