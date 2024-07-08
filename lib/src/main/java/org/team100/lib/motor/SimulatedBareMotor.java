package org.team100.lib.motor;

import org.team100.lib.motor.model.GenericTorqueModel;
import org.team100.lib.telemetry.Logger;
import org.team100.lib.telemetry.Telemetry.Level;
import org.team100.lib.util.Util;

import edu.wpi.first.math.MathUtil;

public class SimulatedBareMotor implements BareMotor, GenericTorqueModel {
    private final Logger m_logger;
    private final double m_freeSpeedRad_S;
    private double m_velocity = 0;

    public SimulatedBareMotor(Logger parent, double freeSpeedRad_S) {
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
    public void setVelocity(double velocity, double accel, double torque) {
        m_velocity = MathUtil.clamp(
                Util.notNaN(velocity), -m_freeSpeedRad_S, m_freeSpeedRad_S);
        m_logger.logDouble(Level.TRACE, "velocity", () -> m_velocity);
    }

    @Override
    public void setPosition(double position, double velocity, double torque) {
        //
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
}
