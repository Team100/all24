package org.team100.lib.motion;

import org.team100.lib.motor.BareMotor;

/**
 * Uses a motor and gears to produce rotational output, e.g. an arm joint.
 * 
 * Motor velocity and accel is higher than mechanism, required torque is lower,
 * using the supplied gear ratio.
 */
public class RotaryMechanism {
    private final BareMotor m_motor;
    private final double m_gearRatio;

    public RotaryMechanism(BareMotor motor, double gearRatio) {
        m_motor = motor;
        m_gearRatio = gearRatio;
    }

    public void setDutyCycle(double output) {
        m_motor.setDutyCycle(output);
    }

    public void setVelocity(
            double outputRad_S,
            double outputAccelRad_S2,
            double outputTorqueNm) {
        m_motor.setVelocity(
                outputRad_S * m_gearRatio,
                outputAccelRad_S2 * m_gearRatio,
                outputTorqueNm / m_gearRatio);
    }

    public void setPosition(
            double outputPositionRad,
            double outputVelocityRad_S,
            double outputTorqueNm) {
        m_motor.setPosition(
                outputPositionRad * m_gearRatio,
                outputVelocityRad_S * m_gearRatio,
                outputTorqueNm / m_gearRatio);
    }

    public void stop() {
        m_motor.stop();
    }

    public void close() {
        m_motor.close();
    }

    public double getVelocityRad_S() {
        return m_motor.getVelocityRad_S() / m_gearRatio;
    }

}
