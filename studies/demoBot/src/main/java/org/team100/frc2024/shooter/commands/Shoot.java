package org.team100.frc2024.shooter.commands;

import org.team100.frc2024.shooter.drumShooter.DrumShooter;
import org.team100.frc2024.shooter.indexer.Indexer;

import edu.wpi.first.wpilibj2.command.Command;

public class Shoot extends Command {
    private final DrumShooter m_shooter;
    private final Indexer m_indexer;

    public Shoot(
            DrumShooter shooter,
            Indexer indexer) {
        m_shooter = shooter;
        m_indexer = indexer;
        addRequirements(m_shooter, m_indexer);
    }

    @Override
    public void initialize() {
        m_shooter.spinUp();
    }

    @Override
    public void execute() {
        if (m_shooter.atVeloctity()) {
            m_indexer.set(1);
        }
    }

    @Override
    public void end(boolean interrupted) {
        m_shooter.stop();
        m_indexer.stop();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
