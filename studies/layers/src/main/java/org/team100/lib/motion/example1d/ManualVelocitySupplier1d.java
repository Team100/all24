package org.team100.lib.motion.example1d;

import java.util.function.DoubleSupplier;

import org.team100.lib.profile.MotionProfile;

/**
 * Supplies raw manual input as velocities.
 */
public class ManualVelocitySupplier1d implements ProfileFollower {
    private final DoubleSupplier m_manual;

    public ManualVelocitySupplier1d(DoubleSupplier manual) {
        m_manual = manual;
    }

    @Override
    public void accept(MotionProfile profile) {
        //
    }

    @Override
    public Double apply(double position_M) {
        return m_manual.getAsDouble();
    }
}
