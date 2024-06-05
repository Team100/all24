package org.team100.commands;

import org.team100.Debug;
import org.team100.subsystems.IndexerSubsystem;

import edu.wpi.first.wpilibj2.command.Command;

/** Run the intake until a note is sensed. */
public class Intake extends Command {

    private final IndexerSubsystem m_indexer;
    private final boolean m_debug;

    public Intake(IndexerSubsystem indexer, boolean debug) {
        m_indexer = indexer;
        m_debug = debug && Debug.enable();
        addRequirements(indexer);
    }

    @Override
    public void execute() {
        if (m_debug)
            System.out.println("Intake");
        m_indexer.intake();
    }

    @Override
    public boolean isFinished() {
        return m_indexer.full();
    }
}
