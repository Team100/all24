package org.team100.lib.telemetry;

import edu.wpi.first.util.function.BooleanConsumer;
import edu.wpi.first.wpilibj.PWM;

/**
 * Turns on a PWM in response to boolean alerts, e.g. from the Monitor class.
 * 
 * I'd suggest wiring the PWM through a motor controller to a car horn.
 */
public class Annunciator implements BooleanConsumer {
    private final PWM m_pwm;

    public Annunciator(int channel) {
        m_pwm = new PWM(channel);
    }

    @Override
    public void accept(boolean value) {
        if (value) {
            m_pwm.setSpeed(1);
        } else {
            m_pwm.setSpeed(0);
        }
    }
}
