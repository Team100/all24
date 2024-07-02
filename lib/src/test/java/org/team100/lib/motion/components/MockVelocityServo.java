package org.team100.lib.motion.components;

import java.util.OptionalDouble;

import org.team100.lib.units.Measure100;

public class MockVelocityServo<T extends Measure100> implements VelocityServo<T> {

    double m_setpoint;

    @Override
    public void reset() {
        //
    }

    @Override
    public void setVelocity(double setpoint) {
        m_setpoint = setpoint;
    }

    @Override
    public OptionalDouble getVelocity() {
        return OptionalDouble.of(m_setpoint);
    }

    @Override
    public void stop() {
        //
    }

    @Override
    public OptionalDouble getDistance() {
        throw new UnsupportedOperationException("Unimplemented method 'getDistance'");
    }

    @Override
    public double getSetpoint() {
        return m_setpoint;
    }
}
