package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;

public class DistancePreset extends Command {
    private final double m_swing;
    private final double m_distance;
    private final double m_grip;
    private final double m_twist;
    private final boolean m_up;
    private final ArmSubsystem m_arm;
    

    public DistancePreset(
            double swing, double distance,
            double grip, double twist, boolean up,
            ArmSubsystem arm) {
        m_swing = swing;
        m_distance = distance;
        m_grip = grip;
        m_twist = twist;
        m_up = up;
        m_arm = arm;
        addRequirements(arm);
    }

    @Override
    public void execute() {
        m_arm.setSwing(m_swing);
        m_arm.setDistance(m_distance, m_up);
        m_arm.setGrip(m_grip);
        m_arm.setTwist(m_twist);
    }

    @Override
    public boolean isFinished() {
        return m_arm.atSetpoint();
    }

}
