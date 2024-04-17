package org.team100.frc2024.motion.intake;

import org.team100.frc2024.RobotState100;
import org.team100.frc2024.RobotState100.IntakeState100;

import edu.wpi.first.wpilibj2.command.Command;

public class RunIntake extends Command {
    private final Intake m_intake;

    public RunIntake(Intake intake) {
        m_intake = intake;
    }

    @Override
    public void initialize() {
        m_intake.resetCurrentCount();
        RobotState100.changeIntakeState(IntakeState100.INTAKE);
    }

    @Override
    public void end(boolean interrupted) {
        RobotState100.changeIntakeState(IntakeState100.STOP);
    }

}
