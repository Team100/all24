package org.team100.lib.motion.mechanism;

import java.util.OptionalDouble;

import org.team100.lib.encoder.IncrementalBareEncoder;
import org.team100.lib.motor.BareMotor;

public class SimpleLinearMechanism implements LinearMechanism {
    private final BareMotor m_motor;
    private final IncrementalBareEncoder m_encoder;
    private final double m_gearRatio;
    private final double m_wheelRadiusM;

    public SimpleLinearMechanism(
            BareMotor motor,
            IncrementalBareEncoder encoder,
            double gearRatio,
            double wheelDiameterM) {
        m_motor = motor;
        m_encoder = encoder;
        m_gearRatio = gearRatio;
        m_wheelRadiusM = wheelDiameterM / 2;
    }

    @Override
    public void setDutyCycle(double output) {
        m_motor.setDutyCycle(output);
    }

    @Override
    public void setForceLimit(double forceN) {
        m_motor.setTorqueLimit(forceN * m_wheelRadiusM / m_gearRatio);
    }

    @Override
    public void setVelocity(
            double outputVelocityM_S,
            double outputAccelM_S2,
            double outputForceN) {
        m_motor.setVelocity(
                (outputVelocityM_S / m_wheelRadiusM) * m_gearRatio,
                (outputAccelM_S2 / m_wheelRadiusM) * m_gearRatio,
                outputForceN * m_wheelRadiusM / m_gearRatio);
    }

    @Override
    public void setPosition(
            double outputPositionM,
            double outputVelocityM_S,
            double outputForceN) {
        m_motor.setPosition(
                (outputPositionM / m_wheelRadiusM) * m_gearRatio,
                (outputVelocityM_S / m_wheelRadiusM) * m_gearRatio,
                outputForceN * m_wheelRadiusM / m_gearRatio);
    }

    @Override
    public OptionalDouble getVelocityM_S() {
        OptionalDouble velocityRad_S = m_encoder.getVelocityRad_S();
        if (velocityRad_S.isEmpty())
            return OptionalDouble.empty();
        return OptionalDouble.of(velocityRad_S.getAsDouble() * m_wheelRadiusM / m_gearRatio);
    }

    @Override
    public OptionalDouble getPositionM() {
        OptionalDouble positionRad = m_encoder.getPositionRad();
        if (positionRad.isEmpty())
            return OptionalDouble.empty();
        return OptionalDouble.of(positionRad.getAsDouble() * m_wheelRadiusM / m_gearRatio);
    }

    @Override
    public void stop() {
        m_motor.stop();
    }

    @Override
    public void close() {
        m_motor.close();
        m_encoder.close();
    }

    @Override
    public void resetEncoderPosition() {
        m_encoder.reset();
    }

    public void periodic() {
        // do some logging
        m_motor.periodic();
        m_encoder.periodic();
    }

}
