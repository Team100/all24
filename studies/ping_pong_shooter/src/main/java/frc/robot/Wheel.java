package frc.robot;

import edu.wpi.first.wpilibj.PWM;
import edu.wpi.first.wpilibj.motorcontrol.PWMMotorController;

public class Wheel extends PWMMotorController {

    protected Wheel(String name, int channel) {
        super(name, channel);
        m_pwm.setPeriodMultiplier(PWM.PeriodMultiplier.k1X);
        m_pwm.setPulseTimeMicroseconds(0);
    }

    public void setRaw(int value) {
        m_pwm.setPulseTimeMicroseconds(value);
        feed();
    }

    public int getRaw() {
        return m_pwm.getPulseTimeMicroseconds();
    }
}
