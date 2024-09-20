package org.team100.lib.encoder;

import java.util.OptionalDouble;

import org.team100.lib.logging.SupplierLogger2;
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

    protected DutyCycleRotaryPositionSensor(
            SupplierLogger2 parent,
            int channel,
            double inputOffset,
            EncoderDrive drive) {
        super(parent, inputOffset, drive);
        m_digitalInput = new DigitalInput(channel);
        m_dutyCycle = new DutyCycle(m_digitalInput);
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
        m_logger.logInt(Level.TRACE, "channel", m_dutyCycle::getSourceChannel);
        double dutyCycle = m_dutyCycle.getOutput();
        m_logger.logDouble(Level.TRACE, "duty cycle", () -> dutyCycle);
        return OptionalDouble.of(dutyCycle);
    }

    private boolean isConnected() {
        return m_dutyCycle.getFrequency() > kFrequencyThreshold;
    }
}
