package org.team100.lib.motor;

/** Motor capable of duty-cycle control. */
public interface DutyCycleMotor100 extends BaseMotor100 {
    /**
     * Open-loop duty cycle control.
     * 
     * @param output in range [-1, 1]
     */
    void setDutyCycle(double output);
}
