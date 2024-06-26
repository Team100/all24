package org.team100.lib.encoder;

import org.team100.lib.units.Measure100;

/**
 * Some kinds of encoders make it easy to set the position offset, but some
 * don't, for example the WPI DutyCycleEncoder and AnalogEncoder classes allow
 * you to set the absolute offset but you can't adjust the turn counter.
 */
public interface SettableEncoder<T extends Measure100> extends Encoder100<T> {
    /**
     * Position in meters or radians
     */
    void setPosition(double position);
}
