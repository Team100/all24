package org.team100.lib.motion.mechanism;

import java.util.OptionalDouble;

/**
 * Proxies a linear mechanism and imposes positional limits, like a software
 * "limit switch." This only makes sense if the "zero" is set correctly.
 * 
 * TODO: make this class aware of whether it's been "homed" correctly.
 */
public class LimitedLinearMechanism implements LinearMechanism {
    private final SimpleLinearMechanism m_delegate;
    private final double m_minPositionM;
    private final double m_maxPositionM;

    public LimitedLinearMechanism(
            SimpleLinearMechanism delegate,
            double minPositionM,
            double maxPositionM) {
        m_delegate = delegate;
        m_minPositionM = minPositionM;
        m_maxPositionM = maxPositionM;
    }

    /** Use for homing. */
    public void setDutyCycleUnlimited(double output) {
        m_delegate.setDutyCycle(output);
    }

    @Override
    public void setDutyCycle(double output) {
        OptionalDouble posOpt = getPositionM();
        if (posOpt.isEmpty()) {
            m_delegate.stop();
            return;
        }
        double posM = posOpt.getAsDouble();
        if (output < 0 && posM < m_minPositionM) {
            m_delegate.stop();
            return;
        }
        if (output > 0 && posM > m_maxPositionM) {
            m_delegate.stop();
            return;
        }
        m_delegate.setDutyCycle(output);
    }

    @Override
    public void setPosition(double outputPositionM, double outputVelocityM_S, double outputForceN) {
        if (outputPositionM < m_minPositionM) {
            m_delegate.stop();
            return;
        }
        if (outputPositionM > m_maxPositionM) {
            m_delegate.stop();
            return;
        }
        m_delegate.setPosition(outputPositionM, outputVelocityM_S, outputForceN);
    }

    /** Use for homing. */
    public void setVelocityUnlimited(double outputVelocityM_S, double outputAccelM_S2, double outputForceN) {
        m_delegate.setVelocity(outputVelocityM_S, outputAccelM_S2, outputForceN);
    }

    @Override
    public void setVelocity(double outputVelocityM_S, double outputAccelM_S2, double outputForceN) {
        OptionalDouble posOpt = getPositionM();
        if (posOpt.isEmpty()) {
            m_delegate.stop();
            return;
        }
        double posM = posOpt.getAsDouble();
        if (outputVelocityM_S < 0 && posM < m_minPositionM) {
            m_delegate.stop();
            return;
        }
        if (outputVelocityM_S > 0 && posM > m_maxPositionM) {
            m_delegate.stop();
            return;
        }
        m_delegate.setVelocity(outputVelocityM_S, outputAccelM_S2, outputForceN);
    }

    @Override
    public void periodic() {
        m_delegate.periodic();
    }

    @Override
    public void setForceLimit(double forceN) {
        m_delegate.setForceLimit(forceN);
    }

    @Override
    public OptionalDouble getVelocityM_S() {
        return m_delegate.getVelocityM_S();
    }

    @Override
    public OptionalDouble getPositionM() {
        return m_delegate.getPositionM();
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

}
