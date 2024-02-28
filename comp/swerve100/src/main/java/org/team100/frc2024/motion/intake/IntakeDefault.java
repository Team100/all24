package org.team100.frc2024.motion.intake;

import org.team100.frc2024.RobotState100;

import edu.wpi.first.wpilibj2.command.Command;

public class IntakeDefault extends Command {
    Intake m_intake;

    public IntakeDefault(Intake intake) {
        m_intake = intake;
        addRequirements(m_intake);
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {

        // if(RobotState100.getIntakeState() == IntakeState100.INTAKE){
        // // System.out.println("WE ARE INTAKING");
        // m_intake.intake();

        // } else if(RobotState100.getIntakeState() == IntakeState100.OUTTAKE){
        // // System.out.println("WE ARE OUTTAKING");
        // m_intake.outtake();

        // }else if(RobotState100.getIntakeState() == IntakeState100.NONE){
        // // System.out.println("WE ARE NONNNEE");
        // m_intake.stop();

        // }

        switch (RobotState100.getIntakeState()) {
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
        //
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
