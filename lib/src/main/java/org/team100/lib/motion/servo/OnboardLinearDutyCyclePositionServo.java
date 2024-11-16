package org.team100.lib.motion.servo;

import java.util.OptionalDouble;

import org.team100.lib.framework.TimedRobot100;
import org.team100.lib.logging.Level;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.logging.LoggerFactory.Control100Logger;
import org.team100.lib.logging.LoggerFactory.DoubleLogger;
import org.team100.lib.logging.LoggerFactory.Model100Logger;
import org.team100.lib.motion.mechanism.LinearMechanism;
import org.team100.lib.profile.Profile100;
import org.team100.lib.state.Control100;
import org.team100.lib.state.Model100;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;

/**
 * Position control using duty cycle feature of linear mechanism
 */
public class OnboardLinearDutyCyclePositionServo implements LinearPositionServo {
    private static final double kV = 0.1;
    private final LinearMechanism m_mechanism;
    private final PIDController m_controller;
    private final Profile100 m_profile;
    // LOGGERS
    private final Model100Logger m_log_goal;
    private final DoubleLogger m_log_measurement;
    private final Control100Logger m_log_setpoint;
    private final DoubleLogger m_log_u_FB;
    private final DoubleLogger m_log_u_FF;
    private final DoubleLogger m_log_u_TOTAL;
    private final DoubleLogger m_log_error;
    private final DoubleLogger m_log_velocity_error;

    private Control100 m_setpoint = new Control100(0, 0);

    public OnboardLinearDutyCyclePositionServo(
            LoggerFactory parent,
            LinearMechanism mechanism,
            PIDController controller,
            Profile100 profile) {
        LoggerFactory child = parent.child(this);
        m_mechanism = mechanism;
        m_controller = controller;
        m_profile = profile;
        m_log_goal = child.model100Logger(Level.TRACE, "goal (m)");
        m_log_measurement = child.doubleLogger(Level.TRACE, "measurement (m)");
        m_log_setpoint = child.control100Logger(Level.TRACE, "setpoint (m)");
        m_log_u_FB = child.doubleLogger(Level.TRACE, "u_FB (duty cycle)");
        m_log_u_FF = child.doubleLogger(Level.TRACE, "u_FF (duty cycle)");
        m_log_u_TOTAL = child.doubleLogger(Level.TRACE, "u_TOTAL (duty cycle)");
        m_log_error = child.doubleLogger(Level.TRACE, "Controller Position Error (m)");
        m_log_velocity_error = child.doubleLogger(Level.TRACE, "Controller Velocity Error (m_s)");
    }

    @Override
    public void reset() {
        m_controller.reset();
        OptionalDouble position = getPosition();
        OptionalDouble velocity = getVelocity();
        if (position.isEmpty() || velocity.isEmpty())
            return;
        m_setpoint = new Control100(position.getAsDouble(), velocity.getAsDouble());
    }

    @Override
    public void setPositionWithVelocity(double goalM, double goalVelocityM_S, double feedForwardTorqueNm) {
        OptionalDouble positionM = m_mechanism.getPositionM();
        if (positionM.isEmpty())
            return;
        double measurementM = positionM.getAsDouble();
        Model100 goal = new Model100(goalM, goalVelocityM_S);
        m_setpoint = m_profile.calculate(TimedRobot100.LOOP_PERIOD_S, m_setpoint.model(), goal);
        double u_FF = kV * m_setpoint.v();
        double u_FB = m_controller.calculate(measurementM, m_setpoint.x());
        double u_TOTAL = MathUtil.clamp(u_FF + u_FB, -1.0, 1.0);
        m_mechanism.setDutyCycle(u_TOTAL);

        m_log_goal.log(() -> goal);
        m_log_measurement.log(() -> measurementM);
        m_log_setpoint.log(() -> m_setpoint);
        m_log_u_FB.log(() -> u_FB);
        m_log_u_FF.log(() -> u_FF);
        m_log_u_TOTAL.log(() -> u_TOTAL);
        m_log_error.log(m_controller::getPositionError);
        m_log_velocity_error.log(m_controller::getVelocityError);
    }

    @Override
    public void setPosition(double goalM, double feedForwardTorqueNm) {
        setPositionWithVelocity(goalM, 0, feedForwardTorqueNm);
    }

    @Override
    public OptionalDouble getPosition() {
        return m_mechanism.getPositionM();
    }

    @Override
    public OptionalDouble getVelocity() {
        return m_mechanism.getVelocityM_S();
    }

    @Override
    public void stop() {
        m_mechanism.stop();
    }

    @Override
    public void close() {
        //
    }

}
