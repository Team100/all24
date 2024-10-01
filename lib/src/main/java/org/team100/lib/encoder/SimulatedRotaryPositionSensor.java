package org.team100.lib.encoder;

import java.util.OptionalDouble;

import org.team100.lib.logging.SupplierLogger2;
import org.team100.lib.logging.SupplierLogger2.DoubleSupplierLogger2;
import org.team100.lib.logging.SupplierLogger2.OptionalDoubleLogger;
import org.team100.lib.motion.mechanism.RotaryMechanism;
import org.team100.lib.telemetry.Telemetry.Level;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Timer;

public class SimulatedRotaryPositionSensor implements RotaryPositionSensor {
    private final RotaryMechanism m_mechanism;
    // LOGGERS
    private final DoubleSupplierLogger2 m_log_position;
    private final OptionalDoubleLogger m_log_rate;

    private double m_positionRad = 0;
    private double m_timeS = Timer.getFPGATimestamp();

    public SimulatedRotaryPositionSensor(
            SupplierLogger2 parent,
            RotaryMechanism motor) {
        SupplierLogger2 child = parent.child(this);
        m_mechanism = motor;
        m_log_position = child.doubleLogger(Level.TRACE, "position");
        m_log_rate = child.optionalDoubleLogger(Level.TRACE, "rate");
    }

    @Override
    public OptionalDouble getPositionRad() {
        double nowS = Timer.getFPGATimestamp();
        double dtS = nowS - m_timeS;
        // motor velocity is rad/s
        OptionalDouble velocityRad_S = m_mechanism.getVelocityRad_S();
        if (velocityRad_S.isEmpty())
            return OptionalDouble.empty();
        m_positionRad += velocityRad_S.getAsDouble() * dtS;
        m_positionRad = MathUtil.angleModulus(m_positionRad);
        m_timeS = nowS;
        m_log_position.log(() -> m_positionRad);
        return OptionalDouble.of(m_positionRad);
    }

    @Override
    public OptionalDouble getRateRad_S() {
        // motor velocity is rad/s
        OptionalDouble m_rate = m_mechanism.getVelocityRad_S();
        if (m_rate.isEmpty())
            return OptionalDouble.empty();
        m_log_rate.log(() -> m_rate);
        return m_rate;
    }

    @Override
    public void close() {
        //
    }

}
