package org.team100.lib.motion.crank;

public class Actuation {
    private double m_velocityM_S;

    public Actuation(double veloctiyM_S) {
        m_velocityM_S = veloctiyM_S;
    }

    public double getVelocityM_S() {
        return m_velocityM_S;
    }
}
