package org.team100.commands;

import org.team100.robot.IndexerSubsystem;
import org.team100.robot.ShooterSubsystem;
import org.team100.sim.Note;

import edu.wpi.first.wpilibj2.command.Command;

/** Take a note from the indexer and shoot it. */
public class ShootCommand extends Command {

    private final IndexerSubsystem m_indexer;
    private final ShooterSubsystem m_shooter;

    public ShootCommand(IndexerSubsystem indexer, ShooterSubsystem shooter) {
        m_indexer = indexer;
        m_shooter = shooter;
        addRequirements(indexer, shooter);
    }

    /**
     * TODO: add spin-up time.
     * TODO: add feeding time.
     */
    @Override
    public void execute() {
        Note note = m_indexer.poll();
        if (note == null)
            return;
        m_shooter.shoot(note);
    }

}
