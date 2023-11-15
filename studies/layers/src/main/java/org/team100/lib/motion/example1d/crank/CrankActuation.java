package org.team100.lib.motion.example1d.crank;

import org.team100.lib.motion.example1d.framework.Actuation;

public class CrankActuation implements Actuation<CrankActuation> {
    private double m_velocityM_S;

    public CrankActuation(double veloctiyM_S) {
        m_velocityM_S = veloctiyM_S;
    }

    @Override
    public CrankActuation getActuation() {
        return this;
    }

    public double getVelocityM_S() {
        return m_velocityM_S;
    }
}
