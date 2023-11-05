package org.team100.lib.motor.turning;

public interface TurningMotor {

    /** @return motor output in range [-1, 1] */
    double get();

    /** @param output motor output in range [-1, 1] */
    void set(double output);

    void setPIDPosition(double output);

    void setPIDVelocity(double output, double outputAccel);
}