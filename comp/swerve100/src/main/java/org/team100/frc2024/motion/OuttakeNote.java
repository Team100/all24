package org.team100.frc2024.motion;

import org.team100.frc2024.motion.indexer.IndexerSubsystem;
import org.team100.frc2024.motion.intake.Intake;

import edu.wpi.first.wpilibj2.command.Command;

public class OuttakeNote extends Command {
    private final Intake m_intake;
    private final IndexerSubsystem m_indexer;

    public OuttakeNote(Intake intake, IndexerSubsystem indexer) {
        m_intake = intake;
        m_indexer = indexer;
        addRequirements(intake, indexer);
    }

    @Override
    public void execute() {
        m_intake.outtake();
        m_indexer.outdex();
    }
}
