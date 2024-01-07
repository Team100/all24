package org.team100.lib.motor;

import org.team100.lib.telemetry.Telemetry;
import org.team100.lib.telemetry.Telemetry.Level;
import org.team100.lib.units.Measure100;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;

/**
 * Very simple simulated motor.
 */
public class SimulatedMotor<T extends Measure100> implements Motor100<T> {
    private final Telemetry t = Telemetry.get();
    private final String m_name;
    private double prevVel = 0;
    private final PIDController controller = new PIDController(1,0,0);
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
        // 100% output => about 6k rpm
        setVelocity(output * 600, 0);
    }

    @Override
    public void stop() {
        m_velocity = 0;
    }
    /**
     * Does not use feedforward as I am not sure how that would work with a fake motor at the moment
     */
    @Override
    public void setVelocity(double velocity, double accel) {
        if (Double.isNaN(velocity))
            throw new IllegalArgumentException("velocity is NaN");
        double output = controller.calculate(m_velocity,velocity);
        m_velocity = prevVel + output;
        prevVel = m_velocity;
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
