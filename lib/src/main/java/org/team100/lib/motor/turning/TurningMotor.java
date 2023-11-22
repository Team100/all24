package org.team100.lib.motor.turning;

public interface TurningMotor {

    /** @return motor output in range [-1, 1] */
    double get();

    /**
     * Open-loop duty cycle control.
     * 
     * @param output motor output in range [-1, 1]
     */
    void setDutyCycle(double output);

    /**
     * Closed-loop velocity control.
     * 
     * @param outputRadiansPerSec velocity in rad/s
     * @param outputAccel         acceleration in rad/s/s
     */
    void setVelocity(double outputRadiansPerSec, double outputAccel);
}