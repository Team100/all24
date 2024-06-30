package org.team100.lib.encoder;

import java.util.OptionalDouble;

import org.team100.lib.units.Measure100;

/**
 * Proxies two encoders and corrects one of them.
 * 
 * The use case is absolute + incremental encoders, in order to do outboard
 * closed-loop position control with only outboard incremental encoders --
 * RoboRIO-attached absolute encoders are the primary, and the incremental
 * encoders are the secondary.
 */
public class CombinedEncoder<T extends Measure100> implements Encoder100<T> {

    private final Encoder100<T> m_primary;
    private final Encoder100<T> m_secondary;

    public CombinedEncoder(Encoder100<T> primary, Encoder100<T> secondary) {
        m_primary = primary;
        m_secondary = secondary;
    }

    @Override
    public OptionalDouble getPosition() {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'getPosition'");
    }

    @Override
    public OptionalDouble getRate() {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'getRate'");
    }

    @Override
    public void reset() {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'reset'");
    }

    @Override
    public void close() {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'close'");
    }

}
