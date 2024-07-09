package org.team100.lib.motion.components;

import java.util.OptionalDouble;

import org.team100.lib.controller.State100;
import org.team100.lib.encoder.CombinedEncoder;
import org.team100.lib.motion.RotaryMechanism;
import org.team100.lib.profile.Profile100;
import org.team100.lib.telemetry.Logger;
import org.team100.lib.telemetry.Telemetry.Level;

import edu.wpi.first.math.MathUtil;

/**
 * Passthrough to outboard closed-loop angular control, using a profile with
 * velocity feedforward, also extra torque (e.g. for gravity).
 * 
 * Must be used with a combined encoder, to "zero" the motor encoder.
 * 
 * TODO: allow other zeroing strategies.
 */
public class OutboardAngularPositionServo implements AngularPositionServo {
    private static final double kDtSec = 0.02;
    private static final double kPositionTolerance = 0.05;
    private static final double kVelocityTolerance = 0.05;
    
    private final Logger m_logger;
    private final RotaryMechanism m_mechanism;
    private final CombinedEncoder m_encoder;
    private final Profile100 m_profile;

    private State100 m_goal = new State100(0, 0);
    private State100 m_setpoint = new State100(0, 0);

    public OutboardAngularPositionServo(
            Logger parent,
            RotaryMechanism mech,
            CombinedEncoder encoder,
            Profile100 profile) {
        m_logger = parent.child(this);
        m_mechanism = mech;
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
    public void setPositionWithVelocity(double goalRad, double goalVelocity, double feedForwardTorqueNm) {
        OptionalDouble positionRad = m_encoder.getPositionRad();
        if (positionRad.isEmpty())
            return;
        double measurementRad = MathUtil.angleModulus(positionRad.getAsDouble());

        // use the modulus closest to the measurement.
        m_goal = new State100(MathUtil.angleModulus(goalRad - measurementRad) + measurementRad, goalVelocity);

        m_setpoint = new State100(
                MathUtil.angleModulus(m_setpoint.x() - measurementRad) + measurementRad,
                m_setpoint.v());

        // NOTE: fixed dt here
        m_setpoint = m_profile.calculate(kDtSec, m_setpoint, m_goal);

        m_mechanism.setPosition(m_setpoint.x(), m_setpoint.v(), feedForwardTorqueNm);

        m_logger.logState100(Level.TRACE, "goal (rad)", () -> m_goal);
        m_logger.logDouble(Level.TRACE, "Feedforward Torque (Nm)", () -> feedForwardTorqueNm);
        m_logger.logDouble(Level.TRACE, "measurement (rad)", () -> measurementRad);
        m_logger.logState100(Level.TRACE, "setpoint (rad)", () -> m_setpoint);
    }

    @Override
    public void setPosition(double goal, double feedForwardTorqueNm) {
        setPositionWithVelocity(goal, 0.0, feedForwardTorqueNm);
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
        OptionalDouble positionRad = m_encoder.getPositionRad();
        if (positionRad.isEmpty())
            return false;
        double positionMeasurementRad = MathUtil.angleModulus(positionRad.getAsDouble());
        OptionalDouble velocityRad_S = m_encoder.getRateRad_S();
        if (velocityRad_S.isEmpty())
            return false;
        double velocityMeasurementRad_S = velocityRad_S.getAsDouble();
        double positionError = m_setpoint.x() - positionMeasurementRad;
        double velocityError = m_setpoint.v() - velocityMeasurementRad_S;
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
        m_mechanism.stop();
    }

    @Override
    public void close() {
        m_mechanism.close();
    }

    @Override
    public State100 getSetpoint() {
        return m_goal;
    }

}
