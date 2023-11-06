package org.team100.lib.motion.example1d;

import java.util.function.DoubleSupplier;

/** Supplies raw manual input as velocities. */
public class ManualVelocitySupplier1d implements DoubleSupplier {
    private final DoubleSupplier m_manual;

    public ManualVelocitySupplier1d(DoubleSupplier manual) {
        m_manual = manual;
    }

    /** @return velocity in meters per second */
    @Override
    public double getAsDouble() {
        return m_manual.getAsDouble();
    }

}
