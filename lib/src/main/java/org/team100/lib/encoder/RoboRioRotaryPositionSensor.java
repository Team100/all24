package org.team100.lib.encoder;

import java.util.OptionalDouble;

import org.team100.lib.logging.SupplierLogger2;
import org.team100.lib.logging.SupplierLogger2.DoubleSupplierLogger2;
import org.team100.lib.logging.SupplierLogger2.OptionalDoubleLogger;
import org.team100.lib.telemetry.Telemetry.Level;
import org.team100.lib.util.Util;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Timer;

/**
 * One of the kinds of absolute rotary position sensors directly connected to
 * the RoboRIO.
 */
public abstract class RoboRioRotaryPositionSensor implements RotaryPositionSensor {
    private static final double kTwoPi = 2.0 * Math.PI;

    private final double m_positionOffset;
    private final EncoderDrive m_drive;
    // LOGGERS
    private final OptionalDoubleLogger m_log_position;
    private final DoubleSupplierLogger2 m_log_position_turns;
    private final DoubleSupplierLogger2 m_log_position_turns_offset;
    private final DoubleSupplierLogger2 m_log_rate;

    private Double m_prevAngleRad = null;
    private Double m_prevTimeS = null;

    protected RoboRioRotaryPositionSensor(
            SupplierLogger2 parent,
            double inputOffset,
            EncoderDrive drive) {
        SupplierLogger2 child = parent.child(this);
        m_positionOffset = Util.inRange(inputOffset, 0.0, 1.0);
        m_drive = drive;
        m_log_position = child.optionalDoubleLogger(Level.TRACE, "position (rad)");
        m_log_position_turns = child.doubleLogger(Level.TRACE, "position (turns)");
        m_log_position_turns_offset = child.doubleLogger(Level.TRACE, "position (turns-offset)");
        m_log_rate = child.doubleLogger(Level.TRACE, "rate (rad)s)");
    }

    /** Implementations should cache this. */
    protected abstract OptionalDouble getRatio();

    protected abstract double m_sensorMin();

    protected abstract double m_sensorMax();

    /** This should be nearly cached. */
    @Override
    public OptionalDouble getPositionRad() {
        OptionalDouble positionRad = getRad();
        m_log_position.log(() -> positionRad);
        return positionRad;
    }

    /** map to full [0,1] */
    protected double mapSensorRange(double pos) {
        // map sensor range
        if (pos < m_sensorMin()) {
            pos = m_sensorMin();
        }
        if (pos > m_sensorMax()) {
            pos = m_sensorMax();
        }
        pos = (pos - m_sensorMin()) / (m_sensorMax() - m_sensorMin());
        return pos;
    }

    /**
     * This should be nearly cached.
     * 
     * @return radians, [-pi, pi]
     */
    protected OptionalDouble getRad() {
        OptionalDouble ratio = getRatio();
        if (ratio.isEmpty())
            return OptionalDouble.empty();

        double posTurns = mapSensorRange(ratio.getAsDouble());
        m_log_position_turns.log(() -> posTurns);

        double turnsMinusOffset = posTurns - m_positionOffset;
        m_log_position_turns_offset.log(() -> turnsMinusOffset);

        switch (m_drive) {
            case DIRECT:
                return OptionalDouble.of(MathUtil.angleModulus(turnsMinusOffset * kTwoPi));
            case INVERSE:
                return OptionalDouble.of(MathUtil.angleModulus(-1.0 * turnsMinusOffset * kTwoPi));
            default:
                throw new IllegalArgumentException();
        }
    }

    /**
     * Nearly cached.
     * 
     * Current rate in rad/s.
     * 
     * This is simply the backward finite difference over one time step.
     * 
     * As such, it is likely to be very noisy.
     * 
     * Use a simple filter if you want a lagged, smoother measurement.
     * 
     * Use a Kalman filter if you can, to reduce the lag.
     */
    @Override
    public OptionalDouble getRateRad_S() {
        OptionalDouble angleRad = getRad();
        if (angleRad.isEmpty())
            return OptionalDouble.empty();
        double timeS = Timer.getFPGATimestamp();
        if (m_prevAngleRad == null) {
            m_prevAngleRad = angleRad.getAsDouble();
            m_prevTimeS = timeS;
            return OptionalDouble.of(0);
        }
        double dxRad = MathUtil.angleModulus(angleRad.getAsDouble() - m_prevAngleRad);
        double dtS = timeS - m_prevTimeS;

        m_prevAngleRad = angleRad.getAsDouble();
        m_prevTimeS = timeS;

        double rateRad_S = dxRad / dtS;
        m_log_rate.log(() -> rateRad_S);
        return OptionalDouble.of(rateRad_S);
    }

}
