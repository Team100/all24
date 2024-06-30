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
     * @param velocity desired velocity in m/s or rad/s.
     * @param accel    desired accel in m/s^2 or rad/s^2
     * @param torque   desired torque in N or Nm, for gravity compensation or
     *                 holding.
     */
    void setVelocity(double velocity, double accel, double torque);
}
