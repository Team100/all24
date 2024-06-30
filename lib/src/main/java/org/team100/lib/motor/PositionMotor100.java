package org.team100.lib.motor;

import org.team100.lib.units.Measure100;

/** Motor capable of closed-loop position control. */
public interface PositionMotor100<T extends Measure100> extends BaseMotor100 {
    /**
     * 
     * @param position
     * @param torque
     */
    void setPosition(double position, double torque);
}
