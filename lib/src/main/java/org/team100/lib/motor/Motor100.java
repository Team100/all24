package org.team100.lib.motor;

import org.team100.lib.dashboard.Glassy;
import org.team100.lib.units.Measure100;

/**
 * Combines all the motor capabilities.
 * 
 * Implementations don't need to cover them all.
 * 
 * TODO: remove Measure100.
 */
public interface Motor100<T extends Measure100> extends Glassy {

    /**
     * Open-loop duty cycle control.
     * 
     * @param output in range [-1, 1]
     */
    void setDutyCycle(double output);

    /**
     * Velocity control with acceleration and holding torque.
     * 
     * Could be open-loop (e.g. "kV") or closed-loop.
     * 
     * Measures here are mechanism measurements, not motor measurements, e.g. output
     * shaft rad/s not motor shaft.
     * 
     * @param velocity m/s or rad/s.
     * @param accel    m/s^2 or rad/s^2.
     * @param torque   N or Nm, for gravity compensation or holding.
     */
    void setVelocity(double velocity, double accel, double torque);

    /**
     * Position control with holding torque.
     * 
     * Measures here are mechanism measurements, not motor measurements, e.g. output
     * shaft position, not motor shaft.
     * 
     * For angular control, revolutions wind up; 0 != 2pi.
     * 
     * @param position meters or radians.
     * @param velocity m/s or rad/s.
     * @param torque   N or Nm, for gravity compensation or holding.
     */
    void setPosition(double position, double velocity, double torque);

    void stop();

    /**
     * For test cleanup.
     */
    void close();

    @Override
    default String getGlassName() {
        return "Motor100";
    }
}
