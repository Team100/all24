package org.team100.lib.motor;

import org.team100.lib.logging.Level;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.logging.LoggerFactory.DoubleLogger;
import org.team100.lib.util.Util;

import edu.wpi.first.math.MathUtil;

public class SimulatedBareMotor implements BareMotor {
    private final double m_freeSpeedRad_S;
    // LOGGERS
    private final DoubleLogger m_log_duty;
    private final DoubleLogger m_log_velocity;

    private double m_velocity = 0;

    public SimulatedBareMotor(LoggerFactory parent, double freeSpeedRad_S) {
        LoggerFactory child = parent.child(this);
        m_freeSpeedRad_S = freeSpeedRad_S;
        m_log_duty = child.doubleLogger(Level.DEBUG, "duty_cycle");
        m_log_velocity = child.doubleLogger(Level.DEBUG, "velocity (rad_s)");
    }

    @Override
    public void setDutyCycle(double dutyCycle) {
        final double output = MathUtil.clamp(
                Util.notNaN(dutyCycle), -1, 1);
        m_log_duty.log(() -> output);
        setVelocity(output * m_freeSpeedRad_S, 0, 0);
    }

    @Override
    public void setVelocity(double velocityRad_S, double accelRad_S2, double torqueNm) {
        m_velocity = MathUtil.clamp(
                Util.notNaN(velocityRad_S), -m_freeSpeedRad_S, m_freeSpeedRad_S);
        m_log_velocity.log(() -> m_velocity);
    }

    @Override
    public void setPosition(double position, double velocity, double torque) {
        //
    }

    /** placeholder */
    @Override
    public double kROhms() {
        return 0.1;
    }

    /** placeholder */
    @Override
    public double kTNm_amp() {
        return 0.02;
    }

    @Override
    public void stop() {
        m_velocity = 0;
    }

    @Override
    public void close() {
        //
    }

    @Override
    public double getVelocityRad_S() {
        return m_velocity;
    }

    @Override
    public void setEncoderPositionRad(double positionRad) {
        //
    }

    @Override
    public void setTorqueLimit(double torqueNm) {
        //
    }

    @Override
    public void periodic() {
        //
    }
}
