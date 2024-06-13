package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;

public class ManualArm extends Command {
    private static final double DEADBAND = 0.05;

    private final XboxController m_controller;
    private final ArmSubsystem m_arm;

    public ManualArm(XboxController controller, ArmSubsystem arm) {
        m_controller = controller;
        m_arm = arm;
        addRequirements(arm);
    }

    @Override
    public void execute() {
        m_arm.addSwing(MathUtil.applyDeadband(m_controller.getLeftX(), DEADBAND));
        m_arm.addBoom(MathUtil.applyDeadband(m_controller.getLeftY(), DEADBAND));
        m_arm.addStick(MathUtil.applyDeadband(m_controller.getRightY(), DEADBAND));
        m_arm.addWrist(MathUtil.applyDeadband(m_controller.getRightX(), DEADBAND));
        m_arm.addGrip(MathUtil.applyDeadband(m_controller.getLeftTriggerAxis()
                - m_controller.getRightTriggerAxis(), DEADBAND));
        double twistSpeed = 0;
        if (m_controller.getLeftBumper()) {
            twistSpeed = 1;
        } else if (m_controller.getRightBumper()) {
            twistSpeed = -1;
        } 
        m_arm.addTwist(twistSpeed);
    }

}
