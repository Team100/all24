package org.team100.lib.motion.mechanism;

import java.util.OptionalDouble;

/**
 * Uses a motor, gears, and a wheel to produce linear output, e.g. a drive wheel
 * or conveyor belt.
 */
public interface LinearMechanism {

    void setDutyCycle(double output);

    void setForceLimit(double forceN);

    void setVelocity(
            double outputVelocityM_S,
            double outputAccelM_S2,
            double outputForceN);

    void setPosition(
            double outputPositionM,
            double outputVelocityM_S,
            double outputForceN);

    OptionalDouble getVelocityM_S();

    OptionalDouble getPositionM();

    void stop();

    void close();

    void resetEncoderPosition();

    void periodic();

}