package org.team100.lib.motion;

import java.util.OptionalDouble;

public interface LinearMechanismInterface {

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

}