package org.team100.lib.motion.components;

import java.util.OptionalDouble;

import org.team100.lib.controller.State100;
import org.team100.lib.encoder.IncrementalBareEncoder;
import org.team100.lib.motion.LinearMechanism;
import org.team100.lib.profile.Profile100;
import org.team100.lib.telemetry.SupplierLogger;
import org.team100.lib.telemetry.Telemetry.Level;

import edu.wpi.first.math.MathUtil;

/**
 * Position control using duty cycle feature of linear mechanism
 */
public class OutboardLinearPositionServo implements LinearPositionServo {
    private final SupplierLogger m_logger;
    private final LinearMechanism m_mechanism;
    private final Profile100 m_profile;
    private final IncrementalBareEncoder m_encoder;
    
    private static final double kDtSec = 0.02;
    private State100 m_goal;
    private State100 m_setpoint = new State100(0, 0);

    public OutboardLinearPositionServo(
            SupplierLogger parent,
            LinearMechanism mechanism,
            IncrementalBareEncoder encoder,
            Profile100 profile) {
        m_logger = parent.child(this);
        m_mechanism = mechanism;
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
    public void setPositionWithVelocity(double goalM, double goalVelocityM_S, double feedForwardTorqueNm) {
        double measurementRad = MathUtil.angleModulus(m_encoder.getPositionRad().getAsDouble());

        m_goal = new State100(goalM, goalVelocityM_S);

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

    public void setVelocity(double goalVelocityM_S, double feedForwardTorqueNm) {
        m_mechanism.setVelocity(goalVelocityM_S, 0, feedForwardTorqueNm);
    }

    @Override
    public OptionalDouble getPosition() {
        return m_encoder.getPositionRad();
    }

    @Override
    public OptionalDouble getVelocity() {
        return m_encoder.getVelocityRad_S();
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
