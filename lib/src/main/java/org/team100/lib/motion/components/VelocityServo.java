package org.team100.lib.motion.components;

import java.util.OptionalDouble;

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

    /**
     * @return Current velocity measurement. Note this can be noisy, maybe filter
     *         it.
     */
    OptionalDouble getVelocity();

    void stop();

    OptionalDouble getDistance();

    /** For testing */
    double getSetpoint();

    @Override
    default String getGlassName() {
        return "VelocityServo";
    }
}