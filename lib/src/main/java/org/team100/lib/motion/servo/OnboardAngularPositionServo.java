package org.team100.lib.motion.servo;

import java.util.OptionalDouble;

import org.team100.lib.encoder.RotaryPositionSensor;
import org.team100.lib.experiments.Experiment;
import org.team100.lib.experiments.Experiments;
import org.team100.lib.framework.TimedRobot100;
import org.team100.lib.logging.Level;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.logging.LoggerFactory.BooleanLogger;
import org.team100.lib.logging.LoggerFactory.Control100Logger;
import org.team100.lib.logging.LoggerFactory.DoubleLogger;
import org.team100.lib.logging.LoggerFactory.Model100Logger;
import org.team100.lib.motion.mechanism.RotaryMechanism;
import org.team100.lib.profile.NullProfile;
import org.team100.lib.profile.Profile100;
import org.team100.lib.state.Control100;
import org.team100.lib.state.Model100;
import org.team100.lib.state.State100;
import org.team100.lib.util.Util;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.LinearFilter;

/**
 * Because the 2025 angular encoder classes do not wind up, this is a version of
 * the position servo that understands that; it's almost a copy of
 * OnboardPositionServo.
 */
public class OnboardAngularPositionServo implements AngularPositionServo {
    private static final double kFeedbackDeadbandRad_S = 0.01;

    private final RotaryMechanism m_mechanism;
    private final RotaryPositionSensor m_positionSensor;
    private final double m_maxVel;
    /**
     * This is positional feedback only, since velocity feedback is handled
     * outboard.
     */
    private final PIDController m_controller;

    /**
     * Smooth out the feedback output.
     * TODO: is this really necessary?
     */
    private final LinearFilter m_filter;

    /** Profile may be updated at runtime. */
    private Profile100 m_profile = new NullProfile();
    // this was Sanjan experimenting in October 2024
    // private ProfileWPI profileTest = new ProfileWPI(40,120);

    private Model100 m_goal = new Model100(0, 0);
    private Control100 m_setpointRad = new Control100(0, 0);

    // LOGGERS
    private final Model100Logger m_log_goal;
    private final DoubleLogger m_log_feedforward_torque;
    private final Model100Logger m_log_measurement;
    private final Control100Logger m_log_setpoint;
    private final DoubleLogger m_log_u_FB;
    private final DoubleLogger m_log_u_FF;
    private final DoubleLogger m_log_u_TOTAL;
    private final DoubleLogger m_log_error;
    private final DoubleLogger m_log_velocity_error;
    private final DoubleLogger m_log_position_tolerance;
    private final DoubleLogger m_log_velocity_tolerance;
    private final BooleanLogger m_log_at_setpoint;

    /**
     * Don't forget to set a profile.
     * TODO: remove maxVel.
     */
    public OnboardAngularPositionServo(
            LoggerFactory parent,
            RotaryMechanism mech,
            RotaryPositionSensor positionSensor,
            double maxVel,
            PIDController controller) {
        LoggerFactory child = parent.child(this);
        m_mechanism = mech;
        m_positionSensor = positionSensor;
        m_maxVel = maxVel;
        m_controller = controller;
        m_controller.setIntegratorRange(0, 0.1);
        m_filter = LinearFilter.singlePoleIIR(0.02, TimedRobot100.LOOP_PERIOD_S);

        m_log_goal = child.model100Logger(Level.TRACE, "goal (rad)");
        m_log_feedforward_torque = child.doubleLogger(Level.TRACE, "Feedforward Torque (Nm)");
        m_log_measurement = child.model100Logger(Level.TRACE, "measurement (rad)");
        m_log_setpoint = child.control100Logger(Level.TRACE, "setpoint (rad)");
        m_log_u_FB = child.doubleLogger(Level.TRACE, "u_FB (rad_s)");
        m_log_u_FF = child.doubleLogger(Level.TRACE, "u_FF (rad_s)");
        m_log_u_TOTAL = child.doubleLogger(Level.TRACE, "u_TOTAL (rad_s)");
        m_log_error = child.doubleLogger(Level.TRACE, "Controller Position Error (rad)");
        m_log_velocity_error = child.doubleLogger(Level.TRACE, "Controller Velocity Error (rad_s)");
        m_log_position_tolerance = child.doubleLogger(Level.TRACE, "Position Tolerance");
        m_log_velocity_tolerance = child.doubleLogger(Level.TRACE, "Velocity Tolerance");
        m_log_at_setpoint = child.booleanLogger(Level.TRACE, "At Setpoint");
    }

    /**
     * It is essential to call this after a period of disuse, to prevent transients.
     * 
     * To prevent oscillation, the previous setpoint is used to compute the profile,
     * but there needs to be an initial setpoint.
     */
    @Override
    public void reset() {
        m_controller.reset();
        OptionalDouble position = getPosition();
        OptionalDouble velocity = getVelocity();
        if (position.isEmpty() || velocity.isEmpty()) {
            Util.warn("OnboardAngularPositionServo: Broken sensor!");
            return;
        }
        m_setpointRad = new Control100(position.getAsDouble(), velocity.getAsDouble());
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
    public void setPositionWithVelocity(
            double goalRad,
            double goalVelocityRad_S,
            double feedForwardTorqueNm) {
        OptionalDouble positionRad = m_positionSensor.getPositionRad();
        // note the mechanism uses the motor's internal encoder which may be only
        // approximately attached to the the output, via backlash and slack, so these
        // two measurements might not be entirely consistent.
        OptionalDouble optVel = m_mechanism.getVelocityRad_S();

        if (positionRad.isEmpty() || optVel.isEmpty()) {
            Util.warn("GravityServo: Broken sensor!");
            return;
        }
        double measurementPositionRad = MathUtil.angleModulus(positionRad.getAsDouble());
        double mechanismVelocityRad_S = optVel.getAsDouble();

        // use the goal nearest to the measurement.
        m_goal = new Model100(MathUtil.angleModulus(goalRad - measurementPositionRad) + measurementPositionRad,
                goalVelocityRad_S);

        // use the setpoint nearest to the measurement.
        m_setpointRad = new Control100(
                MathUtil.angleModulus(m_setpointRad.x() - measurementPositionRad) + measurementPositionRad,
                m_setpointRad.v());

        m_setpointRad = m_profile.calculate(TimedRobot100.LOOP_PERIOD_S, m_setpointRad.model(), m_goal);
        // this was Sanjan experimenting in October 2024
        // m_setpointRad = profileTest.calculate(0.02, m_setpointRad, m_goal);

        final double u_FB;
        if (Experiments.instance.enabled(Experiment.FilterFeedback)) {
            u_FB = MathUtil.applyDeadband(
                    m_filter.calculate(m_controller.calculate(measurementPositionRad,
                            m_setpointRad.x())),
                    kFeedbackDeadbandRad_S,
                    m_maxVel);
        } else {
            u_FB = m_controller.calculate(measurementPositionRad,
                    m_setpointRad.x());
        }


        final double u_FF = m_setpointRad.v();
        // note u_FF is rad/s, so a big number, u_FB should also be a big number.

        final double u_TOTAL = MathUtil.clamp(u_FB + u_FF, -m_maxVel, m_maxVel);

        // stop using the trailing accel, use the setpoint accel instead.
        // m_mechanism.setVelocity(u_TOTAL, accel(u_TOTAL), feedForwardTorqueNm);
        m_mechanism.setVelocity(u_TOTAL, m_setpointRad.a(), feedForwardTorqueNm);

        m_log_goal.log(() -> m_goal);
        m_log_feedforward_torque.log(() -> feedForwardTorqueNm);
        m_log_measurement.log(() -> new Model100(measurementPositionRad, mechanismVelocityRad_S));
        m_log_setpoint.log(() -> m_setpointRad);
        m_log_u_FB.log(() -> u_FB);
        m_log_u_FF.log(() -> u_FF);
        m_log_u_TOTAL.log(() -> u_TOTAL);
        m_log_error.log(m_controller::getPositionError);
        m_log_velocity_error.log(m_controller::getVelocityError);
    }

    @Override
    public void setPosition(double goalRad, double feedForwardTorqueNm) {
        setPositionWithVelocity(goalRad, 0.0, feedForwardTorqueNm);
    }

    /**
     * @return Current position measurement, radians.
     */
    @Override
    public OptionalDouble getPosition() {
        OptionalDouble position = m_positionSensor.getPositionRad();
        if (position.isEmpty())
            return OptionalDouble.empty();
        return OptionalDouble.of(MathUtil.angleModulus(position.getAsDouble()));
    }

    /**
     * @return Current velocity, rad/s.
     */
    @Override
    public OptionalDouble getVelocity() {
        return m_positionSensor.getRateRad_S();
    }

    @Override
    public boolean atSetpoint() {
        boolean atSetpoint = m_controller.atSetpoint();
        m_log_position_tolerance.log(m_controller::getPositionTolerance);
        m_log_velocity_tolerance.log(m_controller::getVelocityTolerance);
        m_log_at_setpoint.log(() -> atSetpoint);
        return atSetpoint;
    }

    @Override
    public void setEncoderPosition(double positionRad) {
        m_mechanism.setEncoderPosition(positionRad);
    }

    @Override
    public boolean atGoal() {
        return atSetpoint()
                && MathUtil.isNear(
                        m_goal.x(),
                        m_setpointRad.x(),
                        m_controller.getPositionTolerance())
                && MathUtil.isNear(
                        m_goal.v(),
                        m_setpointRad.v(),
                        m_controller.getVelocityTolerance());
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
        m_positionSensor.close();
    }

    /** for testing only */
    @Override
    public Control100 getSetpoint() {
        return m_setpointRad;
    }

    @Override
    public void periodic() {
        m_mechanism.periodic();
        m_log_setpoint.log(() -> m_setpointRad);
        m_log_goal.log(() -> m_goal);
    }
}
