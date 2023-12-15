package org.team100.lib.encoder.turning;

import org.team100.lib.encoder.Encoder100;
import org.team100.lib.telemetry.Telemetry;
import org.team100.lib.telemetry.Telemetry.Level;
import org.team100.lib.units.Angle;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.AnalogEncoder;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.Timer;

/**
 * Analog angular encoder used in swerve modules: MA-3 and Thriftybot.
 */
public class AnalogTurningEncoder implements Encoder100<Angle> {
    /** Describes how the encoder angle is linked to the steering angle. */
    public enum Drive {
        /**
         * Encoder moves the same as the module, e.g. via a belt, as in the WCP modules,
         * or via direct drive, as in the SDS modules, or via two gears in the AM
         * modules.
         */
        DIRECT,
        /**
         * Encoder moves opposite to the module, e.g. via a single gear, as in the
         * Team100 adaptation of the AM modules.
         */
        INVERSE
    }

    private final Telemetry t = Telemetry.get();
    private final AnalogInput m_input;
    private final AnalogEncoder m_encoder;
    private final String m_name;

    private Double prevAngle = null;
    private Double prevTime = null;

    /**
     * @param inputOffset unit = turns, i.e. [0,1]
     */
    public AnalogTurningEncoder(
            String name,
            int channel,
            double inputOffset,
            double gearRatio,
            Drive drive) {
        m_input = new AnalogInput(channel);
        m_encoder = new AnalogEncoder(m_input);
        m_encoder.setPositionOffset(inputOffset);
        switch (drive) {
            case DIRECT:
                m_encoder.setDistancePerRotation(2.0 * Math.PI / gearRatio);
                break;
            case INVERSE:
                m_encoder.setDistancePerRotation(2.0 * Math.PI / (-1.0 * gearRatio));
                break;
        }
        m_name = String.format("/%s/Analog Turning Encoder", name);
    }

    @Override
    public double getPosition() {
        t.log(Level.DEBUG, m_name + "/Channel", m_encoder.getChannel());
        t.log(Level.DEBUG, m_name + "/Angle rad", m_encoder.getDistance());
        t.log(Level.DEBUG, m_name + "/Turns", m_encoder.get());
        t.log(Level.DEBUG, m_name + "/Absolute", m_encoder.getAbsolutePosition());
        t.log(Level.DEBUG, m_name + "/Volts", m_input.getVoltage());
        return m_encoder.getDistance();
    }

    /**
     * Trailing unfiltered velocity, likely to be very noisy.
     * 
     * Use a simple filter if you want a lagged, smoother measurement.
     * 
     * Use a Kalman filter if you can, to reduce the lag.
     */
    @Override
    public double getRate() {
        double angle = getPosition();
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

    @Override
    public void reset() {
        m_encoder.reset();
    }

    public void close() {
        m_input.close();
        m_encoder.close();
    }

    /** This is now measured in RADIANS RADIANS RADIANS */
    @Override
    public double getAbsolutePosition() {
        double positionTurns = m_encoder.getAbsolutePosition();
        return MathUtil.angleModulus(2 * Math.PI * positionTurns);
    }
}
