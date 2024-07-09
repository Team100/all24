package org.team100.lib.encoder;

import java.util.OptionalDouble;

import org.team100.lib.telemetry.Logger;
import org.team100.lib.telemetry.Telemetry.Level;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.RobotController;

/**
 * Absolute rotary position sensor using ratiometric analog input.
 */
public class AnalogTurningEncoder extends RoboRioRotaryPositionSensor {
    private final AnalogInput m_input;

    public AnalogTurningEncoder(
            Logger parent,
            int channel,
            double inputOffset,
            EncoderDrive drive) {
        super(parent, inputOffset, drive);
        m_input = new AnalogInput(channel);
    }

    @Override
    protected double m_sensorMin() {
        return 0.0;
    }

    @Override
    protected double m_sensorMax() {
        return 1.0;
    }

    @Override
    public void close() {
        m_input.close();
    }

    @Override
    protected OptionalDouble getRatio() {
        m_logger.logInt(Level.TRACE, "channel", m_input::getChannel);
        double voltage = m_input.getVoltage();
        m_logger.logDouble(Level.TRACE, "voltage", () -> voltage);
        double ratio = voltage / RobotController.getVoltage5V();
        m_logger.logDouble(Level.TRACE, "ratio", () -> ratio);
        return OptionalDouble.of(ratio);
    }
}
