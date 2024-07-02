package org.team100.frc2024.commands;

import org.team100.frc2024.motion.intake.Intake;
import org.team100.frc2024.motion.shooter.DrumShooter;

import edu.wpi.first.wpilibj2.command.Command;

public class Lob extends Command {
    private final Intake m_intake;
    private final DrumShooter m_shooter;

    public Lob(DrumShooter shooter, Intake intake) {
        m_shooter = shooter;
        m_intake = intake;
        addRequirements(m_shooter, m_intake);
    }
    
    @Override
    public void execute() {
        m_shooter.forward();
        m_shooter.setAngle(0.6);
        m_intake.intakeSmart();
    }

}
