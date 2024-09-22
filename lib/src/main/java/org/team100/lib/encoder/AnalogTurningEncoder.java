package org.team100.lib.encoder;

import java.util.OptionalDouble;

import org.team100.lib.logging.SupplierLogger2;
import org.team100.lib.logging.SupplierLogger2.DoubleSupplierLogger2;
import org.team100.lib.logging.SupplierLogger2.IntSupplierLogger2;
import org.team100.lib.telemetry.Telemetry.Level;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.RobotController;

/**
 * Absolute rotary position sensor using ratiometric analog input.
 */
public class AnalogTurningEncoder extends RoboRioRotaryPositionSensor {
    private final AnalogInput m_input;
    // LOGGERS
    private final IntSupplierLogger2 m_log_channel;
    private final DoubleSupplierLogger2 m_log_voltage;
    private final DoubleSupplierLogger2 m_log_ratio;

    public AnalogTurningEncoder(
            SupplierLogger2 parent,
            int channel,
            double inputOffset,
            EncoderDrive drive) {
        super(parent, inputOffset, drive);
        SupplierLogger2 child = parent.child(this);
        m_input = new AnalogInput(channel);
        m_log_channel = child.intLogger(Level.TRACE, "channel");
        m_log_voltage = child.doubleLogger(Level.TRACE, "voltage");
        m_log_ratio = child.doubleLogger(Level.TRACE, "ratio");
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
        m_log_channel.log( m_input::getChannel);
        double voltage = m_input.getVoltage();
        m_log_voltage.log( () -> voltage);
        double ratio = voltage / RobotController.getVoltage5V();
        m_log_ratio.log( () -> ratio);
        return OptionalDouble.of(ratio);
    }
}
