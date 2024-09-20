package org.team100.lib.motor;

import org.team100.lib.logging.SupplierLogger;
import org.team100.lib.telemetry.Telemetry.Level;
import org.team100.lib.util.Util;

import edu.wpi.first.math.MathUtil;

public class SimulatedBareMotor implements BareMotor {
    private final SupplierLogger m_logger;
    private final double m_freeSpeedRad_S;
    private double m_velocity = 0;

    public SimulatedBareMotor(SupplierLogger parent, double freeSpeedRad_S) {
        m_logger = parent.child(this);
        m_freeSpeedRad_S = freeSpeedRad_S;
    }

    @Override
    public void setDutyCycle(double dutyCycle) {
        final double output = MathUtil.clamp(
                Util.notNaN(dutyCycle), -1, 1);
        m_logger.logDouble(Level.TRACE, "duty_cycle", () -> output);
        setVelocity(output * m_freeSpeedRad_S, 0, 0);
    }

    @Override
    public void setVelocity(double velocityRad_S, double accelRad_S2, double torqueNm) {
        m_velocity = MathUtil.clamp(
                Util.notNaN(velocityRad_S), -m_freeSpeedRad_S, m_freeSpeedRad_S);
        m_logger.logDouble(Level.TRACE, "velocity (rad_s)", () -> m_velocity);
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
