package org.team100.lib.motion.example1d.sled;

/** 1d velocity servo */
public class SledActuation {
    private double m_velocityM_S;

    public SledActuation(double v) {
        m_velocityM_S = v;
    }

    public double getVelocityM_S() {
        return m_velocityM_S;
    }

}
