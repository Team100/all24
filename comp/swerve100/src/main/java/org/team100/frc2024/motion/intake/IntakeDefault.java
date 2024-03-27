package org.team100.frc2024.motion.intake;

import org.team100.frc2024.RobotState100;
import org.team100.frc2024.RobotState100.IntakeState100;
import org.team100.lib.telemetry.Telemetry;
import org.team100.lib.telemetry.Telemetry.Level;

import edu.wpi.first.wpilibj2.command.Command;

public class IntakeDefault extends Command {
    private static final Telemetry t = Telemetry.get();

    Intake m_intake;

    public IntakeDefault(Intake intake) {
        m_intake = intake;
        addRequirements(m_intake);
    }

    @Override
    public void initialize() {
        t.log(Level.DEBUG, "IntakeDefault", "command state", "initialize");
    }

    @Override
    public void execute() {
        t.log(Level.DEBUG, "IntakeDefault", "command state", "execute");

        // if(RobotState100.getIntakeState() == IntakeState100.INTAKE){
        // m_intake.intake();

        // } else if(RobotState100.getIntakeState() == IntakeState100.OUTTAKE){
        // m_intake.outtake();

        // }else if(RobotState100.getIntakeState() == IntakeState100.NONE){
        // m_intake.stop();

        // }

        IntakeState100 intakeState = RobotState100.getIntakeState();
        t.log(Level.DEBUG, "IntakeDefault", "state", intakeState);
        switch (intakeState) {
            case INTAKE:
                m_intake.intakeSmart();
                break;
            case OUTTAKE:
                m_intake.outtake();
                break;
            case STOP:
                m_intake.stop();
                break;
            default:

        }

        switch (RobotState100.getFeederState()) {
            case FEED:
                m_intake.runUpper();
                break;
            default:

        }

    }

    @Override
    public void end(boolean interrupted) {
        t.log(Level.DEBUG, "IntakeDefault", "command state", "end");
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
