package org.team100.lib.motion;

import java.util.OptionalDouble;

/**
 * Proxies a linear mechanism and imposes positional limits, like a software
 * "limit switch." This only makes sense if the "zero" is set correctly.
 */
public class LimitedLinearMechanism implements LinearMechanismInterface {

    private final LinearMechanism m_delegate;
    private final double m_minPositionM;
    private final double m_maxPositionM;

    public LimitedLinearMechanism(
            LinearMechanism delegate,
            double minPositionM,
            double maxPositionM) {
        m_delegate = delegate;
        m_minPositionM = minPositionM;
        m_maxPositionM = maxPositionM;
    }

    @Override
    public void setDutyCycle(double output) {
        OptionalDouble posOpt = getPositionM();
        if (posOpt.isEmpty())
            return;
        double posM = posOpt.getAsDouble();
        if (output < 0 && posM < m_minPositionM)
            return;
        if (output > 0 && posM > m_maxPositionM)
            return;
        m_delegate.setDutyCycle(output);
    }

    @Override
    public void setPosition(double outputPositionM, double outputVelocityM_S, double outputForceN) {
        if (outputPositionM < m_minPositionM)
            return;
        if (outputPositionM > m_maxPositionM)
            return;
        m_delegate.setPosition(outputPositionM, outputVelocityM_S, outputForceN);
    }

    @Override
    public void setVelocity(double outputVelocityM_S, double outputAccelM_S2, double outputForceN) {
        OptionalDouble posOpt = getPositionM();
        if (posOpt.isEmpty())
            return;
        double posM = posOpt.getAsDouble();
        if (outputVelocityM_S < 0 && posM < m_minPositionM)
            return;
        if (outputVelocityM_S > 0 && posM > m_maxPositionM)
            return;
        m_delegate.setVelocity(outputVelocityM_S, outputAccelM_S2, outputForceN);
    }

    public void setForceLimit(double forceN) {
        m_delegate.setForceLimit(forceN);
    }

    public OptionalDouble getVelocityM_S() {
        return m_delegate.getVelocityM_S();
    }

    public OptionalDouble getPositionM() {
        return m_delegate.getPositionM();
    }

    public void stop() {
        m_delegate.stop();
    }

    public void close() {
        m_delegate.close();
    }

    public void resetEncoderPosition() {
        m_delegate.resetEncoderPosition();
    }

}
