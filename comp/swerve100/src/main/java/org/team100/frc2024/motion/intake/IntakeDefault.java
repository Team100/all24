package org.team100.frc2024.motion.intake;

import org.team100.frc2024.RobotState100;
import org.team100.frc2024.RobotState100.FeederState100;
import org.team100.frc2024.RobotState100.IntakeState100;

import edu.wpi.first.wpilibj2.command.Command;

public class IntakeDefault extends Command {
    private final Intake m_intake;

    public IntakeDefault(Intake intake) {
        m_intake = intake;
        addRequirements(m_intake);
    }

    @Override
    public void execute() {
        if (RobotState100.getFeederState() == FeederState100.FEED) {
            m_intake.intake();
        } else if (RobotState100.getIntakeState() == IntakeState100.INTAKE) {
            m_intake.intakeSmart();
        } else if (RobotState100.getIntakeState() == IntakeState100.STOP) {
            m_intake.stop();
        }
    }
}
