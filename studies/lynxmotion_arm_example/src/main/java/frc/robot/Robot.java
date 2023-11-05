package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;

public class Robot extends TimedRobot {
    private final Servo m_swing = new Servo(0);
    private final Servo m_boom = new Servo(1);
    private final Servo m_stick = new Servo(2);
    private final Servo m_wrist = new Servo(3);
    private final Servo m_twist = new Servo(4);
    private final Servo m_grip = new Servo(5);
    XboxController m_controller = new XboxController(0);

    private static final double DEADBAND = 0.05;

    @Override
    public void teleopInit() {
        m_swing.setAngle(90);
        m_boom.setAngle(135);
        m_stick.setAngle(90);
        m_wrist.setAngle(90);
        m_twist.setAngle(90); // not enough controller channels, leave twist fixed
        m_grip.setAngle(90);
    }

    @Override
    public void teleopPeriodic() {
        // this is the SAE pattern, see https://en.wikipedia.org/wiki/Excavator_controls
        m_swing.setAngle(m_swing.getAngle() + MathUtil.applyDeadband(m_controller.getLeftX(), DEADBAND));
        m_boom.setAngle(m_boom.getAngle() + MathUtil.applyDeadband(m_controller.getLeftY(), DEADBAND));
        m_stick.setAngle(m_stick.getAngle() + MathUtil.applyDeadband(m_controller.getRightY(), DEADBAND));
        m_wrist.setAngle(m_wrist.getAngle() + MathUtil.applyDeadband(m_controller.getRightX(), DEADBAND));
        m_grip.setAngle(m_grip.getAngle() + MathUtil
                .applyDeadband(m_controller.getLeftTriggerAxis() - m_controller.getRightTriggerAxis(), DEADBAND));
    }
}