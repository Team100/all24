package org.team100.frc2024.motion;

import org.team100.frc2024.RobotState100;
import org.team100.frc2024.RobotState100.IntakeState100;
import org.team100.frc2024.motion.indexer.IndexerSubsystem;
import org.team100.frc2024.motion.intake.Intake;

import edu.wpi.first.wpilibj2.command.Command;

public class IntakeNote extends Command {
    private final Intake m_intake;
    private final IndexerSubsystem m_indexer;

    public IntakeNote(Intake intake, IndexerSubsystem indexer) {
        m_intake = intake;
        m_indexer = indexer;
    }

    @Override
    public void initialize() {
        RobotState100.changeIntakeState(IntakeState100.INTAKE);
    }

    @Override
    public void end(boolean interrupted) {
        RobotState100.changeIntakeState(IntakeState100.STOP);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
