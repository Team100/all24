package org.team100.lib.motion.components;

import java.util.OptionalDouble;

import org.team100.lib.logging.SupplierLogger2;
import org.team100.lib.logging.SupplierLogger2.DoubleSupplierLogger2;
import org.team100.lib.logging.SupplierLogger2.OptionalDoubleLogger;
import org.team100.lib.motion.LinearMechanism;
import org.team100.lib.telemetry.Telemetry.Level;

import edu.wpi.first.wpilibj.Timer;

public class OutboardLinearVelocityServo implements LinearVelocityServo {
    private final SupplierLogger2 m_logger;
    private final LinearMechanism m_motor;
    // LOGGERS
    private final DoubleSupplierLogger2 m_log_setpoint_v;
    private final DoubleSupplierLogger2 m_log_setpoint_a;
    private final OptionalDoubleLogger m_log_velocity;
    private final OptionalDoubleLogger m_log_position;

    // for calculating acceleration
    private double previousSetpoint = 0;
    private double prevTime;
    private double m_setpoint;

    public OutboardLinearVelocityServo(
            SupplierLogger2 parent,
            LinearMechanism motor) {
        m_logger = parent.child(this);
        m_motor = motor;
        m_log_setpoint_v = m_logger.doubleLogger(Level.TRACE, "setpoint v (m_s)");
        m_log_setpoint_a = m_logger.doubleLogger(Level.TRACE, "setpoint a (m_s2)");
        m_log_velocity = m_logger.optionalDoubleLogger(Level.TRACE, "velocity (m_s)");
        m_log_position = m_logger.optionalDoubleLogger(Level.TRACE, "position (m)");
    }

    @Override
    public void reset() {
        prevTime = Timer.getFPGATimestamp();
    }

    @Override
    public void setVelocityM_S(double setpointM_S) {
        setVelocity(setpointM_S, accel(setpointM_S));
    }

    @Override
    public void setVelocity(double setpointM_S, double setpointM_S2) {
        m_setpoint = setpointM_S;
        m_motor.setVelocity(setpointM_S, setpointM_S2, 0);
        m_log_setpoint_v.log(() -> setpointM_S);
        m_log_setpoint_a.log(() -> setpointM_S2);
    }

    /**
     * @return Current velocity measurement. Note this can be noisy, maybe filter
     *         it.
     */
    @Override
    public OptionalDouble getVelocity() {
        OptionalDouble velocityM_S = m_motor.getVelocityM_S();
        m_log_velocity.log(() -> velocityM_S);
        return velocityM_S;
    }

    @Override
    public void stop() {
        m_motor.stop();
    }

    @Override
    public OptionalDouble getDistance() {
        OptionalDouble positionM = m_motor.getPositionM();
        m_log_position.log(() -> positionM);
        return positionM;
    }

    @Override
    public double getSetpoint() {
        return m_setpoint;
    }

    @Override
    public void periodic() {
        m_motor.periodic();
    }

    ////////////////////////////////////////////////

    /**
     * there will be some jitter in dt, which will result in a small amount of
     * jitter in acceleration, and since this is a trailing difference there will be
     * a tiny bit of delay, compared to the actual profile. If this is
     * a problem, rewrite the profile class to expose the acceleration state and use
     * that instead.
     */
    private double accel(double setpoint) {
        double now = Timer.getFPGATimestamp();
        double dt = now - prevTime;
        prevTime = now;
        double accel = (setpoint - previousSetpoint) / dt;
        previousSetpoint = setpoint;
        return accel;
    }
}
