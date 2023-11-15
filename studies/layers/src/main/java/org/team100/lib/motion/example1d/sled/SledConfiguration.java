package org.team100.lib.motion.example1d.sled;


/** Represents the 1d position of the sled in meters. */
public class SledConfiguration {
    private final double m_positionM;

    /** @param positionM sled position in meters */
    public SledConfiguration(double positionM) {
        m_positionM = positionM;
    }

    /** @return sled position in meters */
    public SledConfiguration getConfiguration() {
        return this;
    }

    public double getPositionM() {
        return m_positionM;
    }
    
}
