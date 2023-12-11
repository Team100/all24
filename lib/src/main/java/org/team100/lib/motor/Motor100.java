package org.team100.lib.motor;

public interface Motor100<T> {

    /**
     * @return output in range [-1, 1]
     */
    double get();

    /**
     * Open-loop duty cycle control.
     * 
     * @param output in range [-1, 1]
     */
    void setDutyCycle(double output);

    /**
     * Closed-loop velocity control.
     * 
     * @param velocity in T
     * @param accel in T/s.  TODO: make the implementations actually use accel.
     */
    void setVelocity(double velocity, double accel);

    void stop();
}