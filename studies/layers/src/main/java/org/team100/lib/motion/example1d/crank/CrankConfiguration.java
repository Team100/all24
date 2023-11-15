package org.team100.lib.motion.example1d.crank;


public class CrankConfiguration {
    private double m_crankAngleRad;

    public CrankConfiguration(double crankAngleRad) {
        m_crankAngleRad = crankAngleRad;
    }

    public CrankConfiguration getConfiguration() {
        return this;
    }

    public double getCrankAngleRad() {
        return m_crankAngleRad;
    }
}
