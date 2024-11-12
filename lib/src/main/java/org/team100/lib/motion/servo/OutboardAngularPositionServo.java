package org.team100.lib.motion.servo;

import java.util.OptionalDouble;

import org.team100.lib.encoder.CombinedEncoder;
import org.team100.lib.framework.TimedRobot100;
import org.team100.lib.logging.Level;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.logging.LoggerFactory.Control100Logger;
import org.team100.lib.logging.LoggerFactory.DoubleLogger;
import org.team100.lib.logging.LoggerFactory.Model100Logger;
import org.team100.lib.logging.LoggerFactory.OptionalDoubleLogger;
import org.team100.lib.motion.mechanism.RotaryMechanism;
import org.team100.lib.profile.Profile100;
import org.team100.lib.state.Control100;
import org.team100.lib.state.Model100;
import org.team100.lib.state.State100;

import edu.wpi.first.math.MathUtil;

/**
 * Passthrough to outboard closed-loop angular control, using a profile with
 * velocity feedforward, also extra torque (e.g. for gravity). There's no
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
    private final Model100Logger m_log_goal;
    private final DoubleLogger m_log_ff_torque;
    private final DoubleLogger m_log_measurement;
    private final Control100Logger m_log_setpoint;
    private final OptionalDoubleLogger m_log_position;

    /** Profile may be updated at runtime. */
    private Profile100 m_profile;
    /** Remember that the outboard goal "winds up" i.e. it's not in [-pi,pi] */
    private Model100 m_goal = new Model100(0, 0);
    /** Remember that the outboard setpoint "winds up" i.e. it's not in [-pi,pi] */
    private Control100 m_setpoint = new Control100(0, 0);
    // this was Sanjan experimenting in October 2024
    // private ProfileWPI profileTest = new ProfileWPI(40,120);
    
    /** Don't forget to set a profile. */
    public OutboardAngularPositionServo(
            LoggerFactory parent,
            RotaryMechanism mech,
            CombinedEncoder encoder) {
        LoggerFactory child = parent.child(this);
        m_mechanism = mech;
        m_encoder = encoder;
        m_log_goal = child.model100Logger(Level.TRACE, "goal (rad)");
        m_log_ff_torque = child.doubleLogger(Level.TRACE, "Feedforward Torque (Nm)");
        m_log_measurement = child.doubleLogger(Level.TRACE, "measurement (rad)");
        m_log_setpoint = child.control100Logger(Level.TRACE, "setpoint (rad)");
        m_log_position = child.optionalDoubleLogger(Level.TRACE, "Position");
    }

    @Override
    public void reset() {
        OptionalDouble position = getPosition();
        OptionalDouble velocity = getVelocity();
        if (position.isEmpty() || velocity.isEmpty())
            return;
        m_setpoint = new Control100(position.getAsDouble(), velocity.getAsDouble());
    }

    @Override
    public void setProfile(Profile100 profile) {
        m_profile = profile;
    }

    @Override
    public void setTorqueLimit(double torqueNm) {
        m_mechanism.setTorqueLimit(torqueNm);
    }

    @Override
    public void setEncoderPosition(double value) {
        m_mechanism.setEncoderPosition(value);
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
        m_goal = new Model100(goalErr + unwrappedMeasurementRad, goalVelocity);

        // @sanjan's version from sep 2024 used measurement as setpoint which i think is
        // an error.
        // m_setpoint = new State100(measurementRad, m_setpoint.v());
        // setpoint is [-inf,inf]
        // wrapped setpoint is [-pi,pi]
        double wrappedSetpoint = MathUtil.angleModulus(m_setpoint.x());
        // setpoint err is [-pi,pi]
        double setpointErr = MathUtil.angleModulus(wrappedSetpoint - wrappedMeasurementRad);
        // we're choosing a setpoint that is near the measurement
        m_setpoint = new Control100(setpointErr + unwrappedMeasurementRad, m_setpoint.v());

        // finally compute a new setpoint
        m_setpoint = m_profile.calculate(TimedRobot100.LOOP_PERIOD_S, m_setpoint.model(), m_goal);
        // this was Sanjan experimenting in October 2024
        // m_setpoint = profileTest.calculate(0.02, m_setpoint, m_goal);

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
        double positionMeasurementRad = positionRad.getAsDouble();
        OptionalDouble velocityRad_S = getVelocity();
        if (velocityRad_S.isEmpty())
            return false;
        double velocityMeasurementRad_S = velocityRad_S.getAsDouble();
        double positionError = MathUtil.angleModulus(m_setpoint.x() - positionMeasurementRad);
        double velocityError = m_setpoint.v() - velocityMeasurementRad_S;
        // System.out.println("position error " + positionError + " velocity error " + velocityError);
        return Math.abs(positionError) < kPositionTolerance
                && Math.abs(velocityError) < kVelocityTolerance;
    }

    private boolean setpointAtGoal() {
        double positionError = MathUtil.angleModulus(m_goal.x() - m_setpoint.x());
        double velocityError = m_goal.v() - m_setpoint.v();
        // System.out.println("goal position error " + positionError + " velocity " + velocityError);
        return Math.abs(positionError) < kPositionTolerance
                && Math.abs(velocityError) < kVelocityTolerance;
    }

    @Override
    public boolean atGoal() {
        return atSetpoint() && setpointAtGoal();
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
    public Control100 getSetpoint() {
        // 11/11/24 note this used to return m_goal ?
        return m_setpoint;
    }

    @Override
    public void periodic() {
        m_mechanism.periodic();
        m_encoder.periodic();
    }
}
