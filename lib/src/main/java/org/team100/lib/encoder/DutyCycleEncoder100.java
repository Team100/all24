package org.team100.lib.encoder;

import org.team100.lib.telemetry.Telemetry;
import org.team100.lib.telemetry.Telemetry.Level;
import org.team100.lib.units.Distance100;
import org.team100.lib.util.Names;

import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.Timer;

/**
 * Analog angular encoder used in swerve modules: MA-3 and Thriftybot.
 */
public class DutyCycleEncoder100 implements Encoder100<Distance100> {
    private final Telemetry t = Telemetry.get();
    private final String m_name;
    // private final AnalogInput m_input;
    public final DutyCycleEncoder m_encoder;

    private Double prevAngle = null;
    private Double prevTime = null;

    /** Current position, updated in periodic() */
    private double m_positionRad;
    /** Current velocity, updated in periodic() */
    private double m_rateRad_S;

    private boolean m_reversed;


    /**
     * @param name        may not start with a slash
     * @param channel     roboRIO analog input channel
     * @param inputOffset unit = turns, i.e. [0,1] subtracted from the raw
     *                    measurement
     * @param gearRatio
     * @param drive       polarity
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
        // m_input = new AnalogInput(channel);
        System.out.println("I HAVE BEEN SET " + name);
        m_encoder = new DutyCycleEncoder(channel);
        m_encoder.setPositionOffset(inputOffset);
        m_encoder.setDistancePerRotation(2 * Math.PI);
        m_encoder.setConnectedFrequencyThreshold(1000);
        
        // t.log(Level.DEBUG, m_name, "channel", m_encoder.getChannel());
    }

    @Override
    public Double getPosition() {

        // System.out.println("IS CONNECTED" + m_name + m_encoder.isConnected());
        if(!m_encoder.isConnected()){
            return null;
        }
        if(m_reversed){
            return -m_encoder.getDistance();
        }
        return m_encoder.getDistance();

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
        m_positionRad = 0;
    }

    public void close() {
        // m_input.close();
        m_encoder.close();
    }

    @Override
    public void periodic() {
        updatePosition();
        updateRate();
        t.log(Level.DEBUG, m_name, "position (rad)", m_encoder.getDistance());
        t.log(Level.DEBUG, m_name, "position (turns)", m_encoder.get());
        t.log(Level.DEBUG, m_name, "position (absolute)", m_encoder.getAbsolutePosition());
        // t.log(Level.DEBUG, m_name, "position (volts)", m_input.getVoltage());
    }

    //////////////////////////////////////////////

    private void updatePosition() {
        m_positionRad = m_encoder.get();
    }

    /** Update position before calling this. */
    private void updateRate() {
        double angle = m_positionRad;
        double time = Timer.getFPGATimestamp();
        if (prevAngle == null) {
            prevAngle = angle;
            prevTime = time;
            m_rateRad_S = 0;
            return;
        }
        double dx = angle - prevAngle;
        double dt = time - prevTime;

        prevAngle = angle;
        prevTime = time;

        m_rateRad_S = dx / dt;
    }
}
