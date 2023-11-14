package org.team100.lib.motion.example1d.sled;

import org.team100.lib.motion.example1d.framework.Actuation;

/** 1d velocity servo */
public class SledActuation implements Actuation<SledActuation> {
    private double m_velocityM_S;

    public SledActuation(double v) {
        m_velocityM_S = v;
    }

    @Override
    public SledActuation getActuation() {
        return this;
    }

    public double getVelocityM_S() {
        return m_velocityM_S;
    }

}
