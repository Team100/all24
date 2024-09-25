package org.team100.lib.motion.servo;

import java.util.OptionalDouble;

import org.team100.lib.controller.State100;
import org.team100.lib.framework.TimedRobot100;
import org.team100.lib.motion.mechanism.LinearMechanism;
import org.team100.lib.profile.Profile100;
import org.team100.lib.logging.SupplierLogger2;
import org.team100.lib.logging.SupplierLogger2.DoubleSupplierLogger2;
import org.team100.lib.logging.SupplierLogger2.State100Logger;
import org.team100.lib.telemetry.Telemetry.Level;

/**
 * Position control using the feedback controller in the motor controller hardware
 */
public class OutboardLinearPositionServo implements LinearPositionServo {
    private final LinearMechanism m_mechanism;
    private final Profile100 m_profile;
    
    private State100 m_goal;
    private State100 m_setpoint = new State100(0, 0);

    private final State100Logger m_log_goal;
    private final DoubleSupplierLogger2 m_log_ff_torque;
    private final State100Logger m_log_setpoint;



    public OutboardLinearPositionServo(
            SupplierLogger2 parent,
            LinearMechanism mechanism,
            Profile100 profile) {
        SupplierLogger2 child = parent.child(this);
        m_mechanism = mechanism;
        m_profile = profile;
        m_log_goal = child.state100Logger(Level.TRACE, "goal (rad)");
        m_log_ff_torque = child.doubleLogger(Level.TRACE, "Feedforward Torque (Nm)");
        m_log_setpoint = child.state100Logger(Level.TRACE, "setpoint (rad)");
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
        m_setpoint = m_profile.calculate(TimedRobot100.LOOP_PERIOD_S, m_setpoint, m_goal);

        m_mechanism.setPosition(m_setpoint.x(), m_setpoint.v(), feedForwardTorqueNm);

        m_log_goal.log( () -> m_goal);
        m_log_ff_torque.log(() -> feedForwardTorqueNm);
        m_log_setpoint.log(() -> m_setpoint);
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
