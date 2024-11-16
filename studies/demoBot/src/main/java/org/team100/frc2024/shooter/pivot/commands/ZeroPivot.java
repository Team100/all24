package org.team100.frc2024.shooter.pivot.commands;

import org.team100.frc2024.shooter.pivot.PivotSubsystem;

import edu.wpi.first.wpilibj2.command.Command;

public class ZeroPivot extends Command {

    private final PivotSubsystem m_pivot;
    private double prevAngle;
    private boolean ready;

    public ZeroPivot(PivotSubsystem pivot) {
        m_pivot = pivot;
        addRequirements(m_pivot);
    }

    @Override
    public void initialize() {
        ready = false;
        m_pivot.setTorqueLimit(0.00001);
    }

    @Override
    public void execute() {
        // double asDouble = m_pivot.getAngleRad().getAsDouble();
        // m_pivot.setAngleRad(asDouble - 0.05);
        // if (Math.abs(prevAngle - asDouble) < 0.025) {
        //     m_pivot.setEncoderPosition(Math.PI/2);
        //     ready = true;
        // } 
        // prevAngle = asDouble; 
    }

    @Override
    public void end(boolean interrupted) { 
        m_pivot.setTorqueLimit(1);
        m_pivot.stop();
    }

    @Override
    public boolean isFinished() {
        return ready; 
    }
}
