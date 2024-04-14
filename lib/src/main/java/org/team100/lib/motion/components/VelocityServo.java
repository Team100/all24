package org.team100.lib.motion.components;

import org.team100.lib.dashboard.Glassy;
import org.team100.lib.units.Measure100;

public interface VelocityServo<T extends Measure100> extends Glassy {

    void reset();

    /**
     * Velocity in meters/sec or radians/sec depending on T.
     * 
     * @param setpoint
     */
    void setVelocity(double setpoint);

    /** Direct control for testing. */
    void setDutyCycle(double dutyCycle);

    /**
     * @return Current velocity measurement. Note this can be noisy, maybe filter
     *         it.
     */
    double getVelocity();

    /**
     * @return Current applied torque.  Used for drive/steer decoupling.
     */
    double getTorque();

    void stop();

    double getDistance();

    /** For testing */
    double getSetpoint();

    @Override
    default String getGlassName() {
        return "VelocityServo";
    }
}