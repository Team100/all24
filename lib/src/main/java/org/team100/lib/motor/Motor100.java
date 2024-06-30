package org.team100.lib.motor;

import org.team100.lib.motor.model.TorqueModel;
import org.team100.lib.units.Measure100;

/**
 * The Motor100 class is for calibrated motor commands.
 */
public interface Motor100<T extends Measure100>
        extends DutyCycleMotor100, VelocityMotor100<T>, TorqueModel {

}