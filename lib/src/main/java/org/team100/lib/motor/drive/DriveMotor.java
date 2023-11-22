package org.team100.lib.motor.drive;

public interface DriveMotor {

    /** @return Drive motor output in range [-1, 1] */
    double get();

    /**
     * Open-loop duty cycle control.
     * 
     * @param output Drive motor output in range [-1, 1]
     */
    void setDutyCycle(double output);

    /**
     * Closed-loop velocity control.
     * 
     * @param output velocity in meters/sec.
     */
    void setVelocity(double output);
}