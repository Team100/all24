package org.team100.lib.encoder.turning;

import org.team100.lib.telemetry.Telemetry;

import edu.wpi.first.wpilibj.AnalogEncoder;
import edu.wpi.first.wpilibj.AnalogInput;

public class AnalogTurningEncoder implements TurningEncoder {

    private final Telemetry t = Telemetry.get();

    private final AnalogInput m_input;
    private final AnalogEncoder m_encoder;
    private final String m_name;

    /**
     * @param channel
     * @param inputOffset unit = turns, i.e. [0,1]
     * @param gearRatio
     */
    public AnalogTurningEncoder(
            String name,
            int channel,
            double inputOffset,
            double gearRatio) {
        m_input = new AnalogInput(channel);
        m_encoder = new AnalogEncoder(m_input);
        m_encoder.setPositionOffset(inputOffset);
        m_encoder.setDistancePerRotation(2.0 * Math.PI / gearRatio);
        m_name = String.format("/Analog Turning Encoder %s", name);
    }

    @Override
    public double getAngle() {
        t.log(m_name + "/Channel", m_encoder.getChannel());
        t.log(m_name + "/Angle rad", m_encoder.getDistance());
        t.log(m_name + "/Turns", m_encoder.get());
        t.log(m_name + "/Absolute", m_encoder.getAbsolutePosition());
        t.log(m_name + "/Volts", m_input.getVoltage());
        return m_encoder.getDistance();
    }

    @Override
    public void reset() {
        m_encoder.reset();
    }

    public void close() {
        m_input.close();
        m_encoder.close();
    }
}
