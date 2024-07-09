package org.team100.lib.motion.components;

import java.util.OptionalDouble;
import org.team100.lib.dashboard.Glassy;

/**
 * Represents a servo whose output is measured in linear units -- this is
 * usually relevant for wheeled mechanisms, where the surface speed of the wheel
 * is the important thing. For example, a conveyor belt drive is a spinning
 * thing, but the important thing is the linear movement of the belt.
 * 
 * This is part of the project to remove generics from the motion components; I
 * think generics make things too hard to read.
 */
public interface LinearVelocityServo extends Glassy {
    void reset();

    /** meters/sec */
    void setVelocity(double setpoint);

    /** meters/sec meters/sec_2 */
    void setVelocity(double setpoint, double setpoint_2);

    /** meters/sec. Note this can be noisy, maybe filter it. */
    OptionalDouble getVelocity();

    void stop();

    OptionalDouble getDistance();

    /** For testing */
    double getSetpoint();

    @Override
    default String getGlassName() {
        return "LinearVelocityServo";
    }

}
