package org.team100.lib.encoder;

import java.util.OptionalDouble;

import org.team100.lib.logging.SupplierLogger2;
import org.team100.lib.logging.SupplierLogger2.DoubleSupplierLogger2;
import org.team100.lib.logging.SupplierLogger2.IntSupplierLogger2;
import org.team100.lib.telemetry.Telemetry.Level;
import org.team100.lib.util.Util;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycle;

/**
 * Absolute rotary position sensor using duty cycle input.
 */
public abstract class DutyCycleRotaryPositionSensor extends RoboRioRotaryPositionSensor {
    private static final int kFrequencyThreshold = 1000;

    private final DigitalInput m_digitalInput;
    private final DutyCycle m_dutyCycle;
    private final DoubleSupplierLogger2 m_log_duty;
    private final IntSupplierLogger2 m_log_channel;

    protected DutyCycleRotaryPositionSensor(
            SupplierLogger2 parent,
            int channel,
            double inputOffset,
            EncoderDrive drive) {
        super(parent, inputOffset, drive);
        m_digitalInput = new DigitalInput(channel);
        m_dutyCycle = new DutyCycle(m_digitalInput);
        m_log_duty = m_logger.doubleLogger(Level.TRACE, "duty cycle");
        m_log_channel = m_logger.intLogger(Level.TRACE, "channel");
    }

    @Override
    public void close() {
        m_dutyCycle.close();
        m_digitalInput.close();
    }

    @Override
    protected OptionalDouble getRatio() {
        if (!isConnected()) {
            Util.warn(String.format("encoder %d not connected", m_dutyCycle.getSourceChannel()));
            return OptionalDouble.empty();
        }
        m_log_channel.log( m_dutyCycle::getSourceChannel);
        double dutyCycle = m_dutyCycle.getOutput();
        m_log_duty.log( () -> dutyCycle);
        return OptionalDouble.of(dutyCycle);
    }

    private boolean isConnected() {
        return m_dutyCycle.getFrequency() > kFrequencyThreshold;
    }
}
