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
        m_swing.setAngle(50);
        m_boom.setAngle(70);
        
        m_stick.setAngle(90);
        m_wrist.setAngle(90);
        m_twist.setAngle(0); // not enough controller channels, leave twist fixed
        m_grip.setAngle(180);
    }

    public void origin() {
        m_swing.setAngle(92.3);
        m_boom.setAngle(171.5);
        m_stick.setAngle(165.30);
        m_wrist.setAngle(90);
        m_twist.setAngle(120); // not enough controller channels, leave twist fixed
        m_grip.setAngle(0);
    }

    @Override
    public void teleopPeriodic() {
        if (m_controller.getAButton())
            teleopInit();
        if (m_controller.getBButton())
            origin();
        // this is the SAE pattern, see https://en.wikipedia.org/wiki/Excavator_controls
        m_swing.setAngle(m_swing.getAngle() + MathUtil.applyDeadband(m_controller.getLeftX(), DEADBAND));
        m_boom.setAngle(m_boom.getAngle() + MathUtil.applyDeadband(m_controller.getLeftY(), DEADBAND));
        m_stick.setAngle(m_stick.getAngle() + MathUtil.applyDeadband(m_controller.getRightY(), DEADBAND));
        m_wrist.setAngle(m_wrist.getAngle() + MathUtil.applyDeadband(m_controller.getRightX(), DEADBAND));
        m_grip.setAngle(m_grip.getAngle() + MathUtil
                .applyDeadband(m_controller.getLeftTriggerAxis() - m_controller.getRightTriggerAxis(), DEADBAND));

        System.out.printf("%5.2f %5.2f %5.2f %5.2f %5.2f\n",
                m_swing.getAngle(),
                m_boom.getAngle(),
                m_stick.getAngle(),
                m_wrist.getAngle(),
                m_grip.getAngle());
    }
}