package org.team100.lib.motion.components;

import java.util.OptionalDouble;

import org.team100.lib.controller.State100;
import org.team100.lib.motion.LinearMechanism;
import org.team100.lib.profile.Profile100;
import org.team100.lib.telemetry.SupplierLogger;
import org.team100.lib.telemetry.Telemetry.Level;

/**
 * Position control using the feedback controller in the motor controller hardware
 */
public class OutboardLinearPositionServo implements LinearPositionServo {
    private final SupplierLogger m_logger;
    private final LinearMechanism m_mechanism;
    private final Profile100 m_profile;
    
    private static final double kDtSec = 0.02;
    private State100 m_goal;
    private State100 m_setpoint = new State100(0, 0);

    public OutboardLinearPositionServo(
            SupplierLogger parent,
            LinearMechanism mechanism,
            Profile100 profile) {
        m_logger = parent.child(this);
        m_mechanism = mechanism;
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
    public void setPositionWithVelocity(double goalM, double goalVelocityM_S, double feedForwardTorqueNm) {

        m_goal = new State100(goalM, goalVelocityM_S);

        // NOTE: fixed dt here
        m_setpoint = m_profile.calculate(kDtSec, m_setpoint, m_goal);

        m_mechanism.setPosition(m_setpoint.x(), m_setpoint.v(), feedForwardTorqueNm);

        m_logger.logState100(Level.TRACE, "goal (rad)", () -> m_goal);
        m_logger.logDouble(Level.TRACE, "Feedforward Torque (Nm)", () -> feedForwardTorqueNm);
        m_logger.logState100(Level.TRACE, "setpoint (rad)", () -> m_setpoint);
    }

    @Override
    public void setPosition(double goalM, double feedForwardTorqueNm) {
        setPositionWithVelocity(goalM, 0.0, feedForwardTorqueNm);
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
        m_mechanism.close();
    }

    public State100 getSetpoint() {
        return m_goal;
    }
}
