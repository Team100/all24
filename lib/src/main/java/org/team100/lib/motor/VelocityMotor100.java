package org.team100.lib.motor;

import org.team100.lib.units.Measure100;

/**
 * Motor capable of velocity control, either open-loop (e.g. "kV") or
 * closed-loop.
 */
public interface VelocityMotor100<T extends Measure100> extends BaseMotor100 {
    /**
     * Velocity control with acceleration and holding torque.
     * 
     * Measures here are mechanism measurements, not motor measurements, e.g. output
     * shaft rad/s not motor shaft.
     * 
     * @param velocity m/s or rad/s.
     * @param accel    m/s^2 or rad/s^2.
     * @param torque   N or Nm, for gravity compensation or holding.
     */
    void setVelocity(double velocity, double accel, double torque);
}
