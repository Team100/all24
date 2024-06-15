package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;

public class CoordinatePreset extends Command {
    private final double m_x;
        private final double m_y;

    private final double m_grip;
    private final boolean m_up;
    private final ArmSubsystem m_arm;
    

    public CoordinatePreset(
             double x, double y,
            double grip, boolean up,
            ArmSubsystem arm) {
        m_x = x;
        m_y = y;
        m_grip = grip;
        m_up = up;
        m_arm = arm;
        addRequirements(arm);
    }

    @Override
    public void execute() {
        m_arm.setPosition(m_x, m_y, m_up);
        m_arm.setGrip(m_grip);
        // m_arm.setTwist(m_twist);
    }

    @Override
    public boolean isFinished() {
        boolean atSetpoint = m_arm.atSetpoint();
        // System.out.println("finished " + atSetpoint);
        return atSetpoint;
    }

}
