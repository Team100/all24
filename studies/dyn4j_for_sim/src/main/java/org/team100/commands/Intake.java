package org.team100.commands;

import org.team100.subsystems.IndexerSubsystem;

import edu.wpi.first.wpilibj2.command.Command;

/** Run the intake until a note is sensed. */
public class Intake extends Command {

    private final IndexerSubsystem m_indexer;

    public Intake(IndexerSubsystem indexer) {
        m_indexer = indexer;
        addRequirements(indexer);
    }

    @Override
    public void execute() {
        System.out.println("Intake execute");
        m_indexer.intake();
    }

    @Override
    public boolean isFinished() {
        return m_indexer.full();
    }
}
