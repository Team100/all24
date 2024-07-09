package org.team100.lib.motion;

import java.util.OptionalDouble;

import org.team100.lib.encoder.IncrementalBareEncoder;
import org.team100.lib.motor.BareMotor;

/**
 * Uses a motor and gears to produce rotational output, e.g. an arm joint.
 * 
 * Motor velocity and accel is higher than mechanism, required torque is lower,
 * using the supplied gear ratio.
 * 
 * The included encoder is the incremental motor encoder.
 */
public class RotaryMechanism {
    private final BareMotor m_motor;
    private final IncrementalBareEncoder m_encoder;
    private final double m_gearRatio;

    public RotaryMechanism(BareMotor motor, IncrementalBareEncoder encoder, double gearRatio) {
        m_motor = motor;
        m_encoder = encoder;
        m_gearRatio = gearRatio;
    }

    public void setDutyCycle(double output) {
        m_motor.setDutyCycle(output);
    }

    public void setVelocity(
            double outputRad_S,
            double outputAccelRad_S2,
            double outputTorqueNm) {
        m_motor.setVelocity(
                outputRad_S * m_gearRatio,
                outputAccelRad_S2 * m_gearRatio,
                outputTorqueNm / m_gearRatio);
    }

    public void setPosition(
            double outputPositionRad,
            double outputVelocityRad_S,
            double outputTorqueNm) {
        m_motor.setPosition(
                outputPositionRad * m_gearRatio,
                outputVelocityRad_S * m_gearRatio,
                outputTorqueNm / m_gearRatio);
    }

    public void stop() {
        m_motor.stop();
    }

    public void close() {
        m_motor.close();
    }

    public void resetEncoderPosition() {
        m_encoder.reset();
    }

    public void setEncoderPosition(double positionRad) {
        double motorPositionRad = positionRad * m_gearRatio;
        m_encoder.setEncoderPositionRad(motorPositionRad);
    }

    public OptionalDouble getVelocityRad_S() {
        OptionalDouble velocityRad_S = m_encoder.getVelocityRad_S();
        if (velocityRad_S.isEmpty())
            return OptionalDouble.empty();
        return OptionalDouble.of(velocityRad_S.getAsDouble() / m_gearRatio);
    }

    public OptionalDouble getPositionRad() {
        OptionalDouble positionRad = m_encoder.getPositionRad();
        if (positionRad.isEmpty())
            return OptionalDouble.empty();
        return OptionalDouble.of(positionRad.getAsDouble() / m_gearRatio);
    }

}
