package org.team100.lib.encoder;

import java.util.OptionalDouble;

import org.team100.lib.motor.SimulatedAngularMotor;
import org.team100.lib.telemetry.Logger;
import org.team100.lib.telemetry.Telemetry.Level;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Timer;

public class SimulatedRotaryPositionSensor implements RotaryPositionSensor {
    private final Logger m_logger;
    private final SimulatedAngularMotor m_motor;
    private final double m_reduction;

    private double m_positionRad = 0;
    private double m_timeS = Timer.getFPGATimestamp();

    public SimulatedRotaryPositionSensor(
            Logger parent,
            SimulatedAngularMotor motor,
            double reduction) {
        m_logger = parent.child(this);
        m_motor = motor;
        m_reduction = reduction;
        reset();
    }

    @Override
    public OptionalDouble getPositionRad() {
        double nowS = Timer.getFPGATimestamp();
        double dt = nowS - m_timeS;
        // motor velocity is rad/s
        double m_rate = m_motor.getVelocity() / m_reduction;
        m_positionRad += m_rate * dt;
        m_positionRad = MathUtil.angleModulus(m_positionRad);
        m_timeS = nowS;
        m_logger.logDouble(Level.TRACE, "position", () -> m_positionRad);
        return OptionalDouble.of(m_positionRad);
    }

    @Override
    public OptionalDouble getRateRad_S() {
        // motor velocity is rad/s
        double m_rate = m_motor.getVelocity() / m_reduction;
        m_logger.logDouble(Level.TRACE, "rate", () -> m_rate);
        return OptionalDouble.of(m_rate);
    }

    @Override
    public void reset() {
        m_positionRad = 0;
        m_timeS = Timer.getFPGATimestamp();
    }

    @Override
    public void close() {
        //
    }

}
