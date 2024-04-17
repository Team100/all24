package org.team100.frc2024.motion.intake;

import org.team100.frc2024.RobotState100;
import org.team100.frc2024.RobotState100.IntakeState100;
import org.team100.lib.telemetry.Telemetry;
import org.team100.lib.telemetry.Telemetry.Level;

import edu.wpi.first.wpilibj2.command.Command;

public class IntakeDefault extends Command {
    private static final Telemetry t = Telemetry.get();

    private final Intake m_intake;

    public IntakeDefault(Intake intake) {
        m_intake = intake;
        addRequirements(m_intake);
    }

    @Override
    public void execute() {
        IntakeState100 intakeState = RobotState100.getIntakeState();
        t.log(Level.DEBUG, "IntakeDefault", "state", intakeState);
        switch (intakeState) {
            case INTAKE:
                m_intake.intakeSmart();
                break;
            case STOP:
                m_intake.stop();
                break;
            default:
        }
        switch (RobotState100.getFeederState()) {
            case FEED:
                // m_intake.runUpper();
                m_intake.intake();
                break;
            default:
        }

        switch (RobotState100.getShooterState()) {
            case LOB:
                // m_intake.runUpper();
                m_intake.intakeSmart();
                break;
            default:
        }
    }
}
