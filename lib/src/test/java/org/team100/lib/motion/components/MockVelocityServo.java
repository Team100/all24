package org.team100.lib.motion.components;

import org.team100.lib.units.Measure100;

public class MockVelocityServo<T extends Measure100> implements VelocityServo<T> {

    double m_setpoint;
    double m_dutyCycle;

    @Override
    public void reset() {
        //
    }

    @Override
    public void setVelocity(double setpoint) {
        m_setpoint = setpoint;
    }

    @Override
    public void setDutyCycle(double dutyCycle) {
        m_dutyCycle = dutyCycle;
    }

    @Override
    public double getVelocity() {
        return m_setpoint;
    }

    @Override
    public void stop() {
        //
    }

    @Override
    public double getDistance() {
        throw new UnsupportedOperationException("Unimplemented method 'getDistance'");
    }

    @Override
    public double getSetpoint() {
        return m_setpoint;
    }

    @Override
    public void periodic() {
        //
    }

}
