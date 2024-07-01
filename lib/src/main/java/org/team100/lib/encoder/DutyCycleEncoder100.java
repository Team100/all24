package org.team100.lib.encoder;

import java.util.OptionalDouble;

import org.team100.lib.telemetry.Telemetry;
import org.team100.lib.telemetry.Telemetry.Level;
import org.team100.lib.units.Distance100;
import org.team100.lib.util.Names;
import org.team100.lib.util.Util;

import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.Timer;

/**
 * Duty cycle encoder such as the AMS 5048.
 */
public class DutyCycleEncoder100 implements Encoder100<Distance100> {
    private final Telemetry t = Telemetry.get();
    private final String m_name;
    public final DutyCycleEncoder m_encoder;

    private Double prevAngle = null;
    private Double prevTime = null;

    private boolean m_reversed;

    /**
     * @param name        may not start with a slash
     * @param channel     roboRIO analog input channel
     * @param inputOffset unit = turns, i.e. [0,1] subtracted from the raw
     *                    measurement
     * @param reversed    polarity
     */
    public DutyCycleEncoder100(
            String name,
            int channel,
            double inputOffset,
            boolean reversed) {
        if (name.startsWith("/"))
            throw new IllegalArgumentException();

        m_reversed = reversed;
        m_name = Names.append(name, this);
        m_encoder = new DutyCycleEncoder(channel);
        m_encoder.setPositionOffset(inputOffset);
        m_encoder.setDistancePerRotation(2 * Math.PI);
        m_encoder.setConnectedFrequencyThreshold(1000);
    }

    @Override
    public OptionalDouble getPosition() {
        if (!m_encoder.isConnected()) {
            Util.warn(String.format("encoder %d not connected", m_encoder.getSourceChannel()));
            return OptionalDouble.empty();
        }
        if (m_reversed) {
            return OptionalDouble.of(-1.0 * (m_encoder.getAbsolutePosition() - m_encoder.getPositionOffset())
                    * m_encoder.getDistancePerRotation());
        }
        return OptionalDouble.of(
                (m_encoder.getAbsolutePosition() - m_encoder.getPositionOffset()) * m_encoder.getDistancePerRotation());
    }

    /**
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
    public OptionalDouble getRate() {
        if (!m_encoder.isConnected()) {
            Util.warn(String.format("encoder %d not connected", m_encoder.getSourceChannel()));
            return OptionalDouble.empty();
        }
        return OptionalDouble.of(getRateRad_S());
    }

    @Override
    public void reset() {
        // ALERT! @joel 2/19/24: I think encoder reset changes the internal offset
        // which is never what we want. but this might be wrong
        // for some other reason
        // m_encoder.reset();
    }

    public void close() {
        // m_input.close();
        m_encoder.close();
    }

    //////////////////////////////////////////////

    private double getPositionRad() {
        t.log(Level.DEBUG, m_name, "position (rad)", m_encoder.getDistance());
        t.log(Level.DEBUG, m_name, "position (turns)", m_encoder.get());
        t.log(Level.DEBUG, m_name, "position (absolute)", m_encoder.getAbsolutePosition());
        // should be fast, no cache needed.
        return m_encoder.getDistance();
    }

    /** finite difference == noise! */
    private double getRateRad_S() {
        double angle = getPositionRad();
        double time = Timer.getFPGATimestamp();
        if (prevAngle == null) {
            prevAngle = angle;
            prevTime = time;
            return 0;
        }
        double dx = angle - prevAngle;
        double dt = time - prevTime;

        prevAngle = angle;
        prevTime = time;

        return dx / dt;
    }
}
