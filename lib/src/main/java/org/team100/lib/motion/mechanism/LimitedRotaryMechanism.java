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
public class LimitedRotaryMechanism implements RotaryMechanism {
    final RotaryMechanism m_delegate;
    final double m_minPositionRad;
    final double m_maxPositionRad;

    public LimitedRotaryMechanism(
            RotaryMechanism delegate,
            double minPositionRad,
            double maxPositionRad) {
        m_delegate = delegate;
        m_minPositionRad = minPositionRad;
        m_maxPositionRad = maxPositionRad;
    }

    /** Use for homing. */
    public void setDutyCycleUnlimited(double output) {
        m_delegate.setDutyCycle(output);
    }

    @Override
    public void setDutyCycle(double output) {
        OptionalDouble posOpt = getPositionRad();
        if (posOpt.isEmpty()) {
            m_delegate.stop();
            return;
        }
        double posRad = posOpt.getAsDouble();
        if (output < 0 && posRad < m_minPositionRad) {
            m_delegate.stop();
            return;
        }
        if (output > 0 && posRad > m_maxPositionRad) {
            m_delegate.stop();
            return;
        }
        m_delegate.setDutyCycle(output);
    }

    @Override
    public void setTorqueLimit(double torqueNm) {
        m_delegate.setTorqueLimit(torqueNm);
    }

    /** Use for homing. */
    public void setVelocityUnlimited(double outputVelocityM_S, double outputAccelM_S2, double outputForceN) {
        m_delegate.setVelocity(outputVelocityM_S, outputAccelM_S2, outputForceN);
    }

    @Override
    public void setVelocity(
            double outputRad_S,
            double outputAccelRad_S2,
            double outputTorqueNm) {
        OptionalDouble posOpt = getPositionRad();
        if (posOpt.isEmpty()) {
            m_delegate.stop();
            return;
        }
        double posRad = posOpt.getAsDouble();
        if (outputRad_S < 0 && posRad < m_minPositionRad) {
            m_delegate.stop();
            return;
        }
        if (outputRad_S > 0 && posRad > m_maxPositionRad) {
            m_delegate.stop();
            return;
        }
        m_delegate.setVelocity(
                outputRad_S,
                outputAccelRad_S2,
                outputTorqueNm);
    }

    @Override
    public void setPosition(
            double outputPositionRad,
            double outputVelocityRad_S,
            double outputTorqueNm) {
        if (outputPositionRad < m_minPositionRad) {
            m_delegate.stop();
            return;
        }
        if (outputPositionRad > m_maxPositionRad) {
            m_delegate.stop();
            return;
        }
        m_delegate.setPosition(
                outputPositionRad,
                outputVelocityRad_S,
                outputTorqueNm);
    }

    /** nearly cached */
    @Override
    public OptionalDouble getVelocityRad_S() {
        return m_delegate.getVelocityRad_S();
    }

    /** For checking calibration, very slow, do not use outside tests. */
    @Override
    public double getPositionBlockingRad() {
        return m_delegate.getPositionBlockingRad();
    }

    /** nearly cached */
    @Override
    public OptionalDouble getPositionRad() {
        return m_delegate.getPositionRad();
    }

    @Override
    public void stop() {
        m_delegate.stop();
    }

    @Override
    public void close() {
        m_delegate.close();
    }

    @Override
    public void resetEncoderPosition() {
        m_delegate.resetEncoderPosition();
    }

    /** This can be very slow, only use it on startup. */
    @Override
    public void setEncoderPosition(double positionRad) {
        m_delegate.setEncoderPosition(positionRad);
    }

    @Override
    public void periodic() {
        m_delegate.periodic();
    }

}
