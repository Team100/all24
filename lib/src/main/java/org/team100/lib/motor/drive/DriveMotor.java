package org.team100.lib.motor.drive;

import com.ctre.phoenix.motorcontrol.ControlMode;

public interface DriveMotor {

    /** @return Drive motor output in range [-1, 1] */
    double get();

    /** @param output Drive motor output in range [-1, 1] */
    void set(double output);

    void setPID(ControlMode control, double output);
}