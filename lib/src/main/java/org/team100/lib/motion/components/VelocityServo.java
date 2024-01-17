package org.team100.lib.motion.components;

import org.team100.lib.units.Measure100;

public interface VelocityServo<T extends Measure100> {

    void reset();

    void setVelocity(Double setpoint);

    /** Direct control for testing. */
    void setDutyCycle(double dutyCycle);

    /**
     * @return Current velocity measurement. Note this can be noisy, maybe filter
     *         it.
     */
    double getVelocity();

    void stop();

    double getDistance();

    /** For testing */
    double getSetpoint();

    void periodic();
}