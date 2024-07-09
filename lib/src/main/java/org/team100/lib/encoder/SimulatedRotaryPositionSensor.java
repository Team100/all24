package org.team100.lib.encoder;

import java.util.OptionalDouble;

import org.team100.lib.motion.RotaryMechanism;
import org.team100.lib.telemetry.Logger;
import org.team100.lib.telemetry.Telemetry.Level;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Timer;

public class SimulatedRotaryPositionSensor implements RotaryPositionSensor {
    private final Logger m_logger;
    private final RotaryMechanism m_motor;

    private double m_positionRad = 0;
    private double m_timeS = Timer.getFPGATimestamp();

    public SimulatedRotaryPositionSensor(
            Logger parent,
            RotaryMechanism motor) {
        m_logger = parent.child(this);
        m_motor = motor;
        reset();
    }

    @Override
    public OptionalDouble getPositionRad() {
        double nowS = Timer.getFPGATimestamp();
        double dtS = nowS - m_timeS;
        // motor velocity is rad/s
        OptionalDouble velocityRad_S = m_motor.getVelocityRad_S();
        if (velocityRad_S.isEmpty())
            return OptionalDouble.empty();
        m_positionRad += velocityRad_S.getAsDouble() * dtS;
        m_positionRad = MathUtil.angleModulus(m_positionRad);
        m_timeS = nowS;
        m_logger.logDouble(Level.TRACE, "position", () -> m_positionRad);
        return OptionalDouble.of(m_positionRad);
    }

    @Override
    public OptionalDouble getRateRad_S() {
        // motor velocity is rad/s
        OptionalDouble m_rate = m_motor.getVelocityRad_S();
        if (m_rate.isEmpty())
            return OptionalDouble.empty();
        m_logger.logOptionalDouble(Level.TRACE, "rate", () -> m_rate);
        return m_rate;
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
