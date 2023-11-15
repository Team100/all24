package org.team100.lib.motion.example1d.crank;

import java.util.function.DoubleSupplier;

import org.team100.lib.profile.MotionProfile;

/**
 * Supplies raw manual input as velocities.
 */
public class CrankManualVelocitySupplier1d implements CrankProfileFollower {
    private final DoubleSupplier m_manual;

    public CrankManualVelocitySupplier1d(DoubleSupplier manual) {
        m_manual = manual;
    }

    @Override
    public CrankProfileFollower withProfile(MotionProfile profile) {
        // ignore the profile
        return this;
    }

    @Override
    public CrankWorkstate get() {
        return new CrankWorkstate(m_manual.getAsDouble());
    }
}
