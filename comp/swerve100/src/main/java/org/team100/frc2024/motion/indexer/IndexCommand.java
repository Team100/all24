package org.team100.frc2024.motion.indexer;

import java.util.function.BooleanSupplier;

import org.team100.lib.commands.Command100;

public class IndexCommand extends Command100 {
    private final IndexerSubsystem m_index;
    private final BooleanSupplier m_readyToTurn;
    public IndexCommand(IndexerSubsystem index, BooleanSupplier readyToTurn) {
        m_index = index;
        m_readyToTurn = readyToTurn;
        addRequirements(m_index);
    }

    @Override
    public void initialize100() {}

    @Override
    public void execute100(double dt) {
        if (m_readyToTurn.getAsBoolean()) {
            m_index.index();
        } else {
            m_index.stop();
        } 
    }

    @Override
    public void end(boolean interrupted) {}

    @Override
    public boolean isFinished() {
        return false;
    }
}
