package org.team100.lib.motion;

import org.team100.lib.motor.BareMotor;

/**
 * Uses a motor, gears, and a wheel to produce linear output, e.g. a drive wheel
 * or conveyor belt.
 */
public class LinearMechanism {
    private final BareMotor m_motor;
    private final double m_gearRatio;
    private final double m_wheelRadiusM;

    public LinearMechanism(BareMotor motor, double gearRatio, double wheelDiameterM) {
        m_motor = motor;
        m_gearRatio = gearRatio;
        m_wheelRadiusM = wheelDiameterM / 2;
    }

    public void setDutyCycle(double output) {
        m_motor.setDutyCycle(output);
    }

    public void setVelocity(
            double outputVelocityM_S,
            double outputAccelM_S2,
            double outputForceN) {
        m_motor.setVelocity(
                (outputVelocityM_S / m_wheelRadiusM) * m_gearRatio,
                (outputAccelM_S2 / m_wheelRadiusM) * m_gearRatio,
                outputForceN * m_wheelRadiusM / m_gearRatio);
    }

    public double getVelocityM_S() {
        return m_motor.getVelocityRad_S() * m_wheelRadiusM / m_gearRatio;
    }

    public void setPosition(
            double outputPositionM,
            double outputVelocityM_S,
            double outputForceN) {
        m_motor.setPosition(
                (outputPositionM / m_wheelRadiusM) * m_gearRatio,
                (outputVelocityM_S / m_wheelRadiusM) * m_gearRatio,
                outputForceN * m_wheelRadiusM / m_gearRatio);
    }

    public void stop() {
        m_motor.stop();
    }

    public void close() {
        m_motor.close();
    }

}
