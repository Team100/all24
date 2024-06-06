package org.team100.commands;

import org.team100.subsystems.IndexerSubsystem;

import edu.wpi.first.wpilibj2.command.Command;

public class Outtake extends Command {
    private final IndexerSubsystem m_indexer;

    public Outtake(IndexerSubsystem indexer) {
        m_indexer = indexer;
        addRequirements(indexer);
    }

    @Override
    public void execute() {
        m_indexer.outtake();
    }

    @Override
    public boolean isFinished() {
        return m_indexer.full();
    }

}
