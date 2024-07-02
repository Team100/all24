package org.team100.commands;

import org.team100.Debug;
import org.team100.subsystems.IndexerSubsystem;
import org.team100.subsystems.ShooterSubsystem;

import edu.wpi.first.wpilibj2.command.Command;

/** Take a note from the indexer and shoot it. */
public class LobCommand extends Command {

    private final IndexerSubsystem m_indexer;
    private final ShooterSubsystem m_shooter;
    private final boolean m_debug;

    public LobCommand(
            IndexerSubsystem indexer,
            ShooterSubsystem shooter,
            boolean debug) {
        m_indexer = indexer;
        m_shooter = shooter;
        m_debug = debug && Debug.enable();
        addRequirements(indexer, shooter);
    }

    /**
     * TODO: add spin-up time.
     * TODO: add feeding time.
     */
    @Override
    public void execute() {
        // System.out.println("LobCommand execute");
        // stash the note in the assembly handoff
        if (!m_indexer.towardsShooter()) {
            if (m_debug)
                System.out.println("no note");
            return;
        }

        // take the note from the handoff
        m_shooter.lob();
    }
}
