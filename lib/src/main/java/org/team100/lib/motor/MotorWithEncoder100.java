package org.team100.lib.motor;

import org.team100.lib.encoder.Encoder100;
import org.team100.lib.units.Measure100;

/**
 * Simply combines the motor and encoder interfaces, since some motors have
 * built-in encoders.
 */
public interface MotorWithEncoder100<T extends Measure100> extends Motor100<T>, Encoder100<T> {

}
