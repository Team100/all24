package org.team100.lib.motor;

import org.team100.lib.telemetry.Telemetry;
import org.team100.lib.telemetry.Telemetry.Level;
import org.team100.lib.units.Measure100;

import edu.wpi.first.math.MathUtil;

/**
 * Very simple simulated motor.
 * 
 * TODO: make it more realistic.
 */
public class SimulatedMotor<T extends Measure100> implements Motor100<T> {
    private final Telemetry t = Telemetry.get();
    private final String m_name;

    /**
     * @param name may not start with slash
     */
    public SimulatedMotor(String name) {
        if (name.startsWith("/"))
            throw new IllegalArgumentException();
        m_name = String.format("/%s/Simulated Motor", name);
    }

    private double m_velocity = 0;

    @Override
    public double get() {
        throw new UnsupportedOperationException();
    }

    @Override
    public void setDutyCycle(double output) {
        output = MathUtil.clamp(output, -1, 1);
        t.log(Level.DEBUG, m_name + "/duty_cycle", output);
        // this is an absurd motor model.
        setVelocity(output * 20, 0);
    }

    @Override
    public void stop() {
        m_velocity = 0;
    }

    @Override
    public void setVelocity(double velocity, double accel) {
        if (Double.isNaN(velocity))
            throw new IllegalArgumentException("velocity is NaN");
        m_velocity = velocity;
        // ignore accel
        t.log(Level.DEBUG, m_name + "/velocity", m_velocity);
    }

    public double getVelocity() {
        t.log(Level.DEBUG, m_name + "/velocity", m_velocity);
        return m_velocity;
    }

    @Override
    public void close() {
        //
    }
}
