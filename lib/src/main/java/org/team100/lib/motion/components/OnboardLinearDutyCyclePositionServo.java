package org.team100.lib.motion.components;

import java.util.OptionalDouble;

import org.team100.lib.controller.State100;
import org.team100.lib.logging.SupplierLogger;
import org.team100.lib.motion.LinearMechanism;
import org.team100.lib.profile.Profile100;
import org.team100.lib.telemetry.Telemetry.Level;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;

/**
 * Position control using duty cycle feature of linear mechanism
 */
public class OnboardLinearDutyCyclePositionServo implements LinearPositionServo {
    private static final double kV = 0.1;
    private final SupplierLogger m_logger;
    private final LinearMechanism m_mechanism;
    private final PIDController m_controller;
    private final double m_period;
    private final Profile100 m_profile;

    private State100 m_setpoint = new State100(0, 0);

    public OnboardLinearDutyCyclePositionServo(
            SupplierLogger parent,
            LinearMechanism mechanism,
            PIDController controller,
            double period,
            Profile100 profile) {
        m_logger = parent.child(this);
        m_mechanism = mechanism;
        m_controller = controller;
        m_period = period;
        m_profile = profile;
    }

    @Override
    public void reset() {
        m_controller.reset();
        OptionalDouble position = getPosition();
        OptionalDouble velocity = getVelocity();
        if (position.isEmpty() || velocity.isEmpty())
            return;
        m_setpoint = new State100(position.getAsDouble(), velocity.getAsDouble());
    }

    @Override
    public void setPosition(double goalM) {
        OptionalDouble positionM = m_mechanism.getPositionM();
        if (positionM.isEmpty())
            return;
        double measurementM = positionM.getAsDouble();
        State100 goal = new State100(goalM, 0);
        m_setpoint = m_profile.calculate(m_period, m_setpoint, goal);
        double u_FF = kV * m_setpoint.v();
        double u_FB = m_controller.calculate(measurementM, m_setpoint.x());
        double u_TOTAL = MathUtil.clamp(u_FF + u_FB, -1.0, 1.0);
        m_mechanism.setDutyCycle(u_TOTAL);

        m_logger.logState100(Level.TRACE, "goal (m)", () -> goal);
        m_logger.logDouble(Level.TRACE, "measurement (m)", () -> measurementM);
        m_logger.logState100(Level.TRACE, "setpoint (m)", () -> m_setpoint);
        m_logger.logDouble(Level.TRACE, "u_FB (duty cycle)", () -> u_FB);
        m_logger.logDouble(Level.TRACE, "u_FF (duty cycle)", () -> u_FF);
        m_logger.logDouble(Level.TRACE, "u_TOTAL (duty cycle)", () -> u_TOTAL);
        m_logger.logDouble(Level.TRACE, "Controller Position Error (m)", m_controller::getPositionError);
        m_logger.logDouble(Level.TRACE, "Controller Velocity Error (m_s)", m_controller::getVelocityError);
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
