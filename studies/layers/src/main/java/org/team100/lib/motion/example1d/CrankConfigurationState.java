package org.team100.lib.motion.example1d;

import org.team100.lib.motion.example1d.framework.Configuration;

// TODO: rename to CrankConfiguration
public class CrankConfigurationState implements Configuration<Double> {
    private double m_crankAngleRad;

    public CrankConfigurationState(double crankAngleRad) {
        m_crankAngleRad = crankAngleRad;
    }

    public Double getConfiguration() {
        return m_crankAngleRad;
    }
}
