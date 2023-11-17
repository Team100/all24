package org.team100.lib.motion.crank;

/** Represents state in configuration space. */
public class Configuration {
    private double m_crankAngleRad;

    public Configuration(double crankAngleRad) {
        m_crankAngleRad = crankAngleRad;
    }

    public Configuration getConfiguration() {
        return this;
    }

    public double getCrankAngleRad() {
        return m_crankAngleRad;
    }
}
