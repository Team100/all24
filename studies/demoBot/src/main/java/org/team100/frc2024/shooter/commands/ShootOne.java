package org.team100.frc2024.shooter.commands;

import org.team100.frc2024.shooter.drumShooter.DrumShooter;
import org.team100.frc2024.shooter.indexer.Indexer;

import edu.wpi.first.wpilibj2.command.Command;

public class ShootOne extends Command {
    private final DrumShooter m_shooter;
    private final Indexer m_indexer;
    private final double distanceDeg = 90;

    private double angle;

    public ShootOne(
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
            m_indexer.set(angle + distanceDeg);
        }
    }

    @Override
    public void end(boolean interrupted) {
        m_shooter.stop();
    }

    // @Override
    // public boolean isFinished() {
        // return Math.abs(m_indexer.getAngle() - (angle + distanceDeg)) < 0.05;
    // }
}
