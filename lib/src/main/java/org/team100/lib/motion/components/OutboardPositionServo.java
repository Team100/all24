package org.team100.lib.motion.components;

import java.util.OptionalDouble;

import org.team100.lib.controller.State100;
import org.team100.lib.encoder.CombinedEncoder;
import org.team100.lib.motor.PositionMotor100;
import org.team100.lib.profile.Profile100;
import org.team100.lib.telemetry.Logger;
import org.team100.lib.telemetry.Telemetry.Level;
import org.team100.lib.units.Angle100;

import edu.wpi.first.math.MathUtil;

/**
 * Passthrough to outboard closed-loop angular control, using a profile with
 * velocity feedforward, also extra torque (e.g. for gravity).
 * 
 * Must be used with a combined encoder, to "zero" the motor encoder.
 * 
 * TODO: allow other zeroing strategies.
 * 
 * TODO: change the name to OutboardAngularPositionServo.
 */
public class OutboardPositionServo implements AngularPositionServo {
    private static final double kDtSec = 0.02;
    private static final double kPositionTolerance = 0.05;
    private static final double kVelocityTolerance = 0.05;
    private final Logger m_logger;
    private final PositionMotor100<Angle100> m_motor;
    private final CombinedEncoder m_encoder;
    private final Profile100 m_profile;

    private State100 m_goal = new State100(0, 0);
    private State100 m_setpoint = new State100(0, 0);

    public OutboardPositionServo(
            Logger parent,
            PositionMotor100<Angle100> motor,
            CombinedEncoder encoder,
            Profile100 profile) {
        m_logger = parent.child(this);
        m_motor = motor;
        m_encoder = encoder;
        m_profile = profile;
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
        OptionalDouble position = m_encoder.getPositionRad();
        if (position.isEmpty())
            return;
        double measurement = MathUtil.angleModulus(position.getAsDouble());

        // use the modulus closest to the measurement.
        // note zero velocity in the goal.
        m_goal = new State100(MathUtil.angleModulus(goal - measurement) + measurement, 0.0);

        m_setpoint = new State100(
                MathUtil.angleModulus(m_setpoint.x() - measurement) + measurement,
                m_setpoint.v());

        // NOTE: fixed dt here
        m_setpoint = m_profile.calculate(kDtSec, m_setpoint, m_goal);

        m_motor.setPosition(m_setpoint.x(), m_setpoint.v(), feedForwardTorqueNm);

        m_logger.logDouble(Level.TRACE, "goal", () -> goal);
        m_logger.logDouble(Level.TRACE, "Measurement", () -> measurement);
        m_logger.logState100(Level.TRACE, "Setpoint", () -> m_setpoint);
        m_logger.logDouble(Level.TRACE, "Feedforward Torque", () -> feedForwardTorqueNm);
    }

    @Override
    public OptionalDouble getPosition() {
        return m_encoder.getPositionRad();
    }

    @Override
    public OptionalDouble getVelocity() {
        return m_encoder.getRateRad_S();
    }

    @Override
    public boolean atSetpoint() {
        OptionalDouble position = m_encoder.getPositionRad();
        if (position.isEmpty())
            return false;
        double positionMeasurement = MathUtil.angleModulus(position.getAsDouble());
        OptionalDouble velocity = m_encoder.getRateRad_S();
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
