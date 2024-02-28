package org.team100.lib.encoder;

import org.team100.lib.telemetry.Telemetry;
import org.team100.lib.telemetry.Telemetry.Level;
import org.team100.lib.units.Distance100;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;

/**
 * Analog angular encoder used in swerve modules: MA-3 and Thriftybot.
 */
public class SparkMaxEncoder implements Encoder100<Distance100> {
    private final Telemetry t = Telemetry.get();
    private final String m_name;
    private final RelativeEncoder m_encoder;

    /** Current position, updated in periodic() */
    private double m_positionRad = 0;
    /** Current velocity, updated in periodic() */
    private double m_rateRad_S = 0;

    /**
     * @param name        may not start with a slash
     * @param channel     roboRIO analog input channel
     * @param inputOffset unit = turns, i.e. [0,1] subtracted from the raw
     *                    measurement
     * @param gearRatio
     * @param drive       polarity
     */
    public SparkMaxEncoder(String name, CANSparkMax motor ) {
        m_name = name;
        m_encoder = motor.getEncoder();
        
    }

    @Override
    public double getPosition() {
        return m_encoder.getPosition();
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
    public double getRate() {
        return m_rateRad_S;
    }

    

    @Override
    public void reset() {
        // ALERT!  @joel 2/19/24: I think encoder reset changes the internal offset
        // which is never what we want.  but this might be wrong
        // for some other reason
        // m_encoder.reset();

        m_encoder.setPosition(0);
        
    }

    public void close() {
    }

    @Override
    public void periodic() {
        updatePosition();
        updateRate();
        t.log(Level.DEBUG, m_name, "position (rotations)", m_encoder.getPosition());
    }

    //////////////////////////////////////////////

    private void updatePosition() {
        m_positionRad = m_encoder.getPosition();
    }

    /** Update position before calling this. */
    private void updateRate() {
        
    }
}
