package org.team100.lib.motion.components;

import java.util.OptionalDouble;

import org.team100.lib.controller.State100;
import org.team100.lib.encoder.Encoder100;
import org.team100.lib.motor.PositionMotor100;
import org.team100.lib.telemetry.Telemetry;
import org.team100.lib.telemetry.Telemetry.Level;
import org.team100.lib.units.Measure100;
import org.team100.lib.util.Names;

/**
 * Passthrough to outboard closed-loop position control.
 */
public class OutboardPositionServo<T extends Measure100> implements PositionServo<T> {
    private final Telemetry t = Telemetry.get();
    private final String m_name;
    private final PositionMotor100<T> m_motor;
    private final Encoder100<T> m_encoder;
    private final T m_instance;

    private State100 m_goal = new State100(0, 0);

    public OutboardPositionServo(
            String name,
            PositionMotor100<T> motor,
            Encoder100<T> encoder,
            T instance) {
        m_name = Names.append(name, this);
        m_motor = motor;
        m_encoder = encoder;
        m_instance = instance;
    }

    @Override
    public void reset() {
        //
    }

    @Override
    public void setPosition(double goal, double feedForwardTorqueNm) {
        OptionalDouble position = m_encoder.getPosition();
        if (position.isEmpty())
            return;
        double measurement = m_instance.modulus(position.getAsDouble());

        // use the modulus closest to the measurement.
        // note zero velocity in the goal.
        m_goal = new State100(m_instance.modulus(goal - measurement) + measurement, 0.0);

        m_motor.setPosition(m_goal.x(), 0);
        t.log(Level.TRACE, m_name, "goal", goal);
    }

    @Override
    public OptionalDouble getPosition() {
        return m_encoder.getPosition();
    }

    @Override
    public OptionalDouble getVelocity() {
        return m_encoder.getRate();
    }

    @Override
    public boolean atSetpoint() {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'atSetpoint'");
    }

    @Override
    public boolean atGoal() {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'atGoal'");
    }

    @Override
    public double getGoal() {
        return m_goal.x();
    }

    @Override
    public void stop() {
        m_motor.stop();
    }

    @Override
    public void close() {
        m_motor.close();
    }

    @Override
    public State100 getSetpoint() {
        return m_goal;
    }

}
