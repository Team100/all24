package org.team100.lib.motion.mechanism;

import java.util.OptionalDouble;

/**
 * Uses a motor and gears to produce rotational output, e.g. an arm joint.
 * 
 * Motor velocity and accel is higher than mechanism, required torque is lower,
 * using the supplied gear ratio.
 * 
 * The included encoder is the incremental motor encoder.
 */
public interface RotaryMechanism {
    
    void setDutyCycle(double output);

    void setTorqueLimit(double torqueNm);

    void setVelocity(
            double outputRad_S,
            double outputAccelRad_S2,
            double outputTorqueNm);

    void setPosition(
            double outputPositionRad,
            double outputVelocityRad_S,
            double outputTorqueNm);

    /** nearly cached */
    OptionalDouble getVelocityRad_S();

    /** For checking calibration, very slow, do not use outside tests. */
    double getPositionBlockingRad();

    /** nearly cached */
    OptionalDouble getPositionRad();

    void stop();

    void close();

    void resetEncoderPosition();

    /** This can be very slow, only use it on startup. */
    void setEncoderPosition(double positionRad);

    void periodic();

}
