package frc.robot;

import edu.wpi.first.wpilibj.PWM;
import edu.wpi.first.wpilibj.motorcontrol.PWMMotorController;

public class Feed extends PWMMotorController {
    protected Feed(String name, int channel) {
        super(name, channel);
        m_pwm.setBoundsMicroseconds(1720, 1520, 1500, 1480, 1280); // parallax360
        m_pwm.setPeriodMultiplier(PWM.PeriodMultiplier.k4X);
        m_pwm.setSpeed(0.0);
        m_pwm.setZeroLatch();
    }
}
