package org.team100.lib.motion.example1d;

import java.util.function.DoubleSupplier;

import org.team100.lib.motion.example1d.framework.Workstate;

/**
 * Supplies raw manual input as velocities.
 */
public class ManualVelocitySupplier1d implements ProfileFollower {
    private final DoubleSupplier m_manual;

    public ManualVelocitySupplier1d(DoubleSupplier manual) {
        m_manual = manual;
    }


    @Override
    public Workstate<Double> apply(Workstate<Double> position_M) {
        return new CrankWorkstate(m_manual.getAsDouble());

    }
}
