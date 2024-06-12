package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;

public class Preset extends Command {
    private static final double kTolerance = 5;
    private final double m_swing;
    private final double m_boom;
    private final double m_stick;
    private final double m_wrist;
    private final double m_grip;
    private final ArmSubsystem m_arm;

    public Preset(
            double swing, double boom, double stick,
            double wrist, double grip,
            ArmSubsystem arm) {
        m_swing = swing;
        m_boom = boom;
        m_stick = stick;
        m_wrist = wrist;
        m_grip = grip;
        m_arm = arm;
        addRequirements(arm);
    }

    @Override
    public void execute() {
        m_arm.setSwing(m_swing);
        m_arm.setBoom(m_boom);
        m_arm.setStick(m_stick);
        m_arm.setWrist(m_wrist);
        m_arm.setGrip(m_grip);
    }

    @Override
    public boolean isFinished() {
        return Math.abs(m_swing - m_arm.getSwing()) < kTolerance
                && Math.abs(m_boom - m_arm.getBoom()) < kTolerance
                && Math.abs(m_stick - m_arm.getStick()) < kTolerance
                && Math.abs(m_wrist - m_arm.getWrist()) < kTolerance
                && Math.abs(m_grip - m_arm.getGrip()) < kTolerance;
    }

}
