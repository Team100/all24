package org.team100.lib.motor;

import org.team100.lib.motor.model.GenericTorqueModel;
import org.team100.lib.telemetry.Telemetry;
import org.team100.lib.telemetry.Telemetry.Level;
import org.team100.lib.units.Measure100;
import org.team100.lib.util.Names;

import edu.wpi.first.math.MathUtil;

/**
 * Very simple simulated motor. Use whatever unit you want: if you set a
 * velocity then you get it back, immediately. If you want to use duty cycle,
 * you should choose a reasonable free speed.
 * 
 * A Neo goes about 6000 rpm, or 100 rev/s, or about 600 rad/s.
 */
public class SimulatedMotor<T extends Measure100> implements Motor100<T>, GenericTorqueModel {
    private final Telemetry.Logger t;
    private final String m_name;
    private final double m_freeSpeed;
    private double m_velocity = 0;

    /**
     * @param name may not start with slash
     * @param kV   velocity units at full output
     */
    public SimulatedMotor(String name, double freeSpeed) {
        m_name = Names.append(name, this);
        t = Telemetry.get().logger(m_name);
        m_freeSpeed = freeSpeed;
    }

    @Override
    public void setDutyCycle(double dutyCycle) {
        final double output = MathUtil.clamp(dutyCycle, -1, 1);
        t.logDouble(Level.TRACE, "duty_cycle", () -> output);
        setVelocity(output * m_freeSpeed, 0, 0);
    }

    @Override
    public void stop() {
        m_velocity = 0;
    }

    /**
     * Velocity only.
     */
    @Override
    public void setVelocity(double velocity, double accel, double torque) {
        if (Double.isNaN(velocity))
            throw new IllegalArgumentException("velocity is NaN");
        m_velocity = velocity;
        t.logDouble(Level.TRACE, "velocity", () -> m_velocity);
    }

    public double getVelocity() {
        return m_velocity;
    }

    @Override
    public void close() {
        //
    }
}
