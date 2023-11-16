package org.team100.lib.motion.crank;


public class CrankActuation {
    private double m_velocityM_S;

    public CrankActuation(double veloctiyM_S) {
        m_velocityM_S = veloctiyM_S;
    }

    public double getVelocityM_S() {
        return m_velocityM_S;
    }
}
