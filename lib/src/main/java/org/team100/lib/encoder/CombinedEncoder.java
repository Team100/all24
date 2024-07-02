package org.team100.lib.encoder;

import java.util.OptionalDouble;

import org.team100.lib.units.Measure100;

/**
 * Proxies two encoders and corrects the position of one of them every time you
 * get it. Also falls back to the secondary if the primary fails.
 * 
 * The use case is absolute + incremental encoders, in order to do outboard
 * closed-loop position control with only outboard incremental encoders --
 * RoboRIO-attached absolute encoders are the primary, and the incremental
 * encoders are the secondary.
 */
public class CombinedEncoder<T extends Measure100> implements Encoder100<T> {
    private final Encoder100<T> m_primary;
    private final double m_authority;
    private final SettableEncoder<T> m_secondary;

    /**
     * 
     * @param primary
     * @param authority how to discount the primary updates: 1.0 = it's truth, 0.0 =
     *                  ignore it.
     * @param secondary
     */
    public CombinedEncoder(
            Encoder100<T> primary,
            double authority,
            SettableEncoder<T> secondary) {
        if (authority < 0)
            throw new IllegalArgumentException("authority must be >= 0");
        if (authority > 1)
            throw new IllegalArgumentException("authority must be <= 1");
        m_primary = primary;
        m_authority = authority;
        m_secondary = secondary;
    }

    @Override
    public OptionalDouble getPosition() {
        OptionalDouble optPrimaryPosition = m_primary.getPosition();
        if (optPrimaryPosition.isPresent()) {
            // Adjust the secondary.
            double primaryPosition = optPrimaryPosition.getAsDouble();
            double secondaryPosition = m_secondary.getPosition().orElse(primaryPosition);
            double correctedPosition = m_authority * primaryPosition + (1.0 - m_authority) * secondaryPosition;
            m_secondary.setPosition(correctedPosition);
            return OptionalDouble.of(correctedPosition);
        }
        // Primary is broken, maybe the secondary is still working.
        return m_secondary.getPosition();
    }

    @Override
    public OptionalDouble getRate() {
        OptionalDouble primaryRate = m_primary.getRate();
        if (primaryRate.isPresent()) {
            // Rate cannot be corrected so just return the primary value.
            return primaryRate;
        }
        // Primary is broken, maybe the secondary is still working.
        return m_secondary.getRate();
    }

    @Override
    public void reset() {
        m_primary.reset();
        m_secondary.reset();
    }

    @Override
    public void close() {
        m_primary.close();
        m_secondary.close();
    }

}
