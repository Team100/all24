package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;

public class DistancePreset extends Command {
    private final double m_swing;
    private final double m_distance;
    private final double m_grip;
    private final ArmSubsystem m_arm;

    public DistancePreset(
            double swing, double distance, double grip,
            ArmSubsystem arm) {
        m_swing = swing;
        m_distance = distance;
        m_grip = grip;
        m_arm = arm;
        addRequirements(arm);
    }

    @Override
    public void execute() {
        m_arm.setSwing(m_swing);
        m_arm.setDistance(m_distance);
        m_arm.setGrip(m_grip);
    }

    @Override
    public boolean isFinished() {
        return m_arm.atSetpoint();
    }

}
