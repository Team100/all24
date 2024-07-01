package org.team100.lib.motor;

import org.team100.lib.units.Measure100;

/** Motor capable of closed-loop position control. */
public interface PositionMotor100<T extends Measure100> extends BaseMotor100 {
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
}
