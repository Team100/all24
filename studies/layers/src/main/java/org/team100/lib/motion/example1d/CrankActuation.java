package org.team100.lib.motion.example1d;

import org.team100.lib.motion.example1d.framework.Actuation;

public class CrankActuation implements Actuation<Double> {
    private double m_velocityM_S;

    public CrankActuation(double veloctiyM_S) {
        m_velocityM_S = veloctiyM_S;
    }

    public Double getActuation() {
        return m_velocityM_S;
    }
}
