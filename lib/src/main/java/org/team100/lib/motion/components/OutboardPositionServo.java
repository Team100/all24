package org.team100.lib.motion.components;

import java.util.OptionalDouble;

import org.team100.lib.controller.State100;
import org.team100.lib.encoder.Encoder100;
import org.team100.lib.motor.PositionMotor100;
import org.team100.lib.profile.Profile100;
import org.team100.lib.telemetry.Telemetry;
import org.team100.lib.telemetry.Telemetry.Level;
import org.team100.lib.units.Measure100;
import org.team100.lib.util.Names;

import edu.wpi.first.math.MathUtil;

/**
 * Passthrough to outboard closed-loop position control, using a profile with
 * velocity feedforward, also extra torque (e.g. for gravity).
 */
public class OutboardPositionServo<T extends Measure100> implements PositionServo<T> {
    private static final double kDtSec = 0.02;
    private static final double kPositionTolerance = 0.05;
    private static final double kVelocityTolerance = 0.05;
    private final Telemetry t = Telemetry.get();
    private final String m_name;
    private final PositionMotor100<T> m_motor;
    private final Encoder100<T> m_encoder;
    private final Profile100 m_profile;
    private final T m_instance;

    private State100 m_goal = new State100(0, 0);
    private State100 m_setpoint = new State100(0, 0);

    public OutboardPositionServo(
            String name,
            PositionMotor100<T> motor,
            Encoder100<T> encoder,
            Profile100 profile,
            T instance) {
        m_name = Names.append(name, this);
        m_motor = motor;
        m_encoder = encoder;
        m_profile = profile;
        m_instance = instance;
    }

    @Override
    public void reset() {
        OptionalDouble position = getPosition();
        OptionalDouble velocity = getVelocity();
        if (position.isEmpty() || velocity.isEmpty())
            return;
        m_setpoint = new State100(position.getAsDouble(), velocity.getAsDouble());
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

        m_setpoint = new State100(
                m_instance.modulus(m_setpoint.x() - measurement) + measurement,
                m_setpoint.v());

        // NOTE: fixed dt here
        m_setpoint = m_profile.calculate(kDtSec, m_setpoint, m_goal);

        m_motor.setPosition(m_setpoint.x(), m_setpoint.v(), feedForwardTorqueNm);

        t.log(Level.TRACE, m_name, "goal", goal);
        t.log(Level.DEBUG, m_name, "Measurement", measurement);
        t.log(Level.DEBUG, m_name, "Setpoint", m_setpoint);
        t.log(Level.TRACE, m_name, "Feedforward Torque", feedForwardTorqueNm);
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
        OptionalDouble position = m_encoder.getPosition();
        if (position.isEmpty())
            return false;
        double positionMeasurement = m_instance.modulus(position.getAsDouble());
        OptionalDouble velocity = m_encoder.getRate();
        if (velocity.isEmpty())
            return false;
        double velocityMeasurement = velocity.getAsDouble();
        double positionError = m_setpoint.x() - positionMeasurement;
        double velocityError = m_setpoint.v() - velocityMeasurement;
        return Math.abs(positionError) < kPositionTolerance
                && Math.abs(velocityError) < kVelocityTolerance;
    }

    @Override
    public boolean atGoal() {
        return atSetpoint()
                && MathUtil.isNear(
                        m_goal.x(),
                        m_setpoint.x(),
                        kPositionTolerance)
                && MathUtil.isNear(
                        m_goal.v(),
                        m_setpoint.v(),
                        kVelocityTolerance);
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
