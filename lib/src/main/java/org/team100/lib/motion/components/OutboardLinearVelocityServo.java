package org.team100.lib.motion.components;

import java.util.OptionalDouble;

import org.team100.lib.motion.LinearMechanism;
import org.team100.lib.telemetry.Logger;
import org.team100.lib.telemetry.Telemetry.Level;

import edu.wpi.first.wpilibj.Timer;

public class OutboardLinearVelocityServo implements LinearVelocityServo {

    private final Logger m_logger;
    private final LinearMechanism m_motor;

    // for calculating acceleration
    private double previousSetpoint = 0;
    private double prevTime;
    private double m_setpoint;

    public OutboardLinearVelocityServo(
            Logger parent,
            LinearMechanism motor) {
        m_logger = parent.child(this);
        m_motor = motor;
    }

    @Override
    public void reset() {
        prevTime = Timer.getFPGATimestamp();
    }

    @Override
    public void setVelocity(double setpointM_S) {
        setVelocity(setpointM_S, accel(setpointM_S));
    }

    @Override
    public void setVelocity(double setpointM_S, double setpointM_S2) {
        m_setpoint = setpointM_S;
        m_motor.setVelocity(setpointM_S, setpointM_S2, 0);
        m_logger.logDouble(Level.TRACE, "setpoint v (m_s)", () -> setpointM_S);
        m_logger.logDouble(Level.TRACE, "setpoint a (m_s2)", () -> setpointM_S2);
    }

    /**
     * @return Current velocity measurement. Note this can be noisy, maybe filter
     *         it.
     */
    @Override
    public OptionalDouble getVelocity() {
        return m_motor.getVelocityM_S();
    }

    @Override
    public void stop() {
        m_motor.stop();
    }

    @Override
    public OptionalDouble getDistance() {
        return m_motor.getPositionM();
    }

    @Override
    public double getSetpoint() {
        return m_setpoint;
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
