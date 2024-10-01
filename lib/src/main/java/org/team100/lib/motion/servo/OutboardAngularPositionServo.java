package org.team100.lib.motion.servo;

import java.util.OptionalDouble;

import org.team100.lib.encoder.CombinedEncoder;
import org.team100.lib.framework.TimedRobot100;
import org.team100.lib.logging.Level;
import org.team100.lib.logging.SupplierLogger2;
import org.team100.lib.logging.SupplierLogger2.DoubleSupplierLogger2;
import org.team100.lib.logging.SupplierLogger2.OptionalDoubleLogger;
import org.team100.lib.logging.SupplierLogger2.State100Logger;
import org.team100.lib.motion.mechanism.RotaryMechanism;
import org.team100.lib.profile.Profile100;
import org.team100.lib.state.State100;

import edu.wpi.first.math.MathUtil;

/**
 * Passthrough to outboard closed-loop angular control, using a profile with
 * velocity feedforward, also extra torque (e.g. for gravity).  There's no
 * feedback at this level, and no feedforward calculation either, that's
 * delegated to the mechanism.
 * 
 * Must be used with a combined encoder, to "zero" the motor encoder.
 * 
 * TODO: allow other zeroing strategies.
 */
public class OutboardAngularPositionServo implements AngularPositionServo {
    private static final double kPositionTolerance = 0.05;
    private static final double kVelocityTolerance = 0.05;

    private final RotaryMechanism m_mechanism;
    private final CombinedEncoder m_encoder;

    // LOGGERS
    private final State100Logger m_log_goal;
    private final DoubleSupplierLogger2 m_log_ff_torque;
    private final DoubleSupplierLogger2 m_log_measurement;
    private final State100Logger m_log_setpoint;
    private final OptionalDoubleLogger m_log_position;

    /** Profile may be updated at runtime. */
    private Profile100 m_profile;

    private State100 m_goal = new State100(0, 0);
    private State100 m_setpoint = new State100(0, 0);

    /** Don't forget to set a profile. */
    public OutboardAngularPositionServo(
            SupplierLogger2 parent,
            RotaryMechanism mech,
            CombinedEncoder encoder) {
        SupplierLogger2 child = parent.child(this);
        m_mechanism = mech;
        m_encoder = encoder;
        m_log_goal = child.state100Logger(Level.TRACE, "goal (rad)");
        m_log_ff_torque = child.doubleLogger(Level.TRACE, "Feedforward Torque (Nm)");
        m_log_measurement = child.doubleLogger(Level.TRACE, "measurement (rad)");
        m_log_setpoint = child.state100Logger(Level.TRACE, "setpoint (rad)");
        m_log_position = child.optionalDoubleLogger(Level.TRACE, "Position");
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
    public void setProfile(Profile100 profile) {
        m_profile = profile;
    }

    @Override
    public void setTorqueLimit(double torqueNm) {
        m_mechanism.setTorqueLimit(torqueNm);
    }

    /** The outboard measurement does not wrap, but the goal does */
    @Override

    public void setPositionWithVelocity(double wrappedGoalRad, double goalVelocity, double feedForwardTorqueNm) {
        // goal is [-pi,pi]
        // but measurement is [-inf,inf]

        OptionalDouble positionRad = m_encoder.getPositionRad();
        m_log_position.log(() -> positionRad);
 
        if (positionRad.isEmpty())
            return;

        // unwrapped measurement is [-inf,inf]
        double unwrappedMeasurementRad = positionRad.getAsDouble();
        // wrapped measurement is [-pi,pi]
        double wrappedMeasurementRad = MathUtil.angleModulus(unwrappedMeasurementRad);

        // err is [-pi,pi]
        double goalErr = MathUtil.angleModulus(wrappedGoalRad - wrappedMeasurementRad);

        // choose a goal which is near the measurement
        // apply error to unwrapped measurement, so goal is [-inf, inf]
        m_goal = new State100(goalErr + unwrappedMeasurementRad, goalVelocity);

        // @sanjan's version from sep 2024 used measurement as setpoint which i think is
        // an error.
        // m_setpoint = new State100(measurementRad, m_setpoint.v());
        // setpoint is [-inf,inf]
        // wrapped setpoint is [-pi,pi]
        double wrappedSetpoint = MathUtil.angleModulus(m_setpoint.x());
        // setpoint err is [-pi,pi]
        double setpointErr = MathUtil.angleModulus(wrappedSetpoint - wrappedMeasurementRad);
        // we're choosing a setpoint that is near the measurement
        m_setpoint = new State100(setpointErr + unwrappedMeasurementRad, m_setpoint.v());

        // finally compute a new setpoint
        m_setpoint = m_profile.calculate(TimedRobot100.LOOP_PERIOD_S, m_setpoint, m_goal);

        m_mechanism.setPosition(m_setpoint.x(), m_setpoint.v(), feedForwardTorqueNm);

        m_log_goal.log(() -> m_goal);
        m_log_ff_torque.log(() -> feedForwardTorqueNm);
        m_log_measurement.log(() -> unwrappedMeasurementRad);
        m_log_setpoint.log(() -> m_setpoint);
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
        OptionalDouble positionRad = getPosition();
        if (positionRad.isEmpty())
            return false;
        double positionMeasurementRad = MathUtil.angleModulus(positionRad.getAsDouble());
        OptionalDouble velocityRad_S = getVelocity();
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

    @Override
    public void periodic() {
        m_mechanism.periodic();
        m_encoder.periodic();
    }
}
