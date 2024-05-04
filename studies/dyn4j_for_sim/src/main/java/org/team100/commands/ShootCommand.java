package org.team100.commands;

import org.team100.Debug;
import org.team100.subsystems.IndexerSubsystem;
import org.team100.subsystems.ShooterSubsystem;

import edu.wpi.first.wpilibj2.command.Command;

/** Take a note from the indexer and shoot it. */
public class ShootCommand extends Command {

    private final IndexerSubsystem m_indexer;
    private final ShooterSubsystem m_shooter;
    private final boolean m_debug;

    public ShootCommand(IndexerSubsystem indexer, ShooterSubsystem shooter, boolean debug) {
        m_indexer = indexer;
        m_shooter = shooter;
        m_debug = debug;
        addRequirements(indexer, shooter);
    }

    /**
     * TODO: add spin-up time.
     * TODO: add feeding time.
     */
    @Override
    public void execute() {
        if (m_debug && Debug.print())
            System.out.print("ShootCommand\n");
        // stash the note in the assembly handoff
        if (!m_indexer.towardsShooter()) {
            System.out.println("no note");
            return;
        }

        // take the note from the handoff
        m_shooter.shoot();
    }

    /**
     * This version finishes immediately, which is clearly wrong.
     * TODO: add spinning, feeding delays.
     */
    @Override
    public boolean isFinished() {
        return true;
    }
}
