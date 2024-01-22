// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.team100.frc2024.motion.indexer;

import org.team100.frc2024.motion.amp.AmpSubsystem;
import org.team100.frc2024.motion.shooter.Shooter;
import org.team100.lib.commands.Command100;

public class IndexCommand extends Command100 {
    /** Creates a new index. */
    AmpSubsystem m_amp;
    IndexerSubsystem m_index;
    Shooter m_shooter;
    double m_desiredShooterVelocity;

    public IndexCommand(AmpSubsystem amp, IndexerSubsystem index, Shooter shooter, double desiredShooterVelocity) {
        m_index = index;
        m_shooter = shooter;
        m_amp = amp;
        m_desiredShooterVelocity = desiredShooterVelocity;
        addRequirements(m_index);
        // Use addRequirements() here to declare subsystem dependencies.
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize100() {}

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute100(double dt) {
        //TODO Get the real value here, should just take a few seconds, put the real amp arm in the correct pos and get the value from glass in rots 
        double ampMidPos = 1;
        if (m_amp.getLeftAmpPosition() > ampMidPos || (m_shooter.getFirstRollerVelocity() > m_desiredShooterVelocity
                && m_shooter.getSecondRollerVelocity() > m_desiredShooterVelocity)) {
            m_index.setDrive(3.5);
        } else {
            m_index.setDrive(0);
        }
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {}

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}
