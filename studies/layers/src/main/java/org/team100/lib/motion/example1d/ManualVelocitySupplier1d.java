package org.team100.lib.motion.example1d;

import java.util.function.DoubleFunction;
import java.util.function.DoubleSupplier;

import org.team100.lib.motion.example1d.framework.Workstate;
import org.team100.lib.profile.MotionProfile;

/**
 * Supplies raw manual input as velocities.
 */
public class ManualVelocitySupplier1d<T extends Workstate<T>> implements ProfileFollower<T> {
    private final DoubleSupplier m_manual;
    private final DoubleFunction<T> m_maker;

    public ManualVelocitySupplier1d(DoubleSupplier manual, DoubleFunction<T> maker) {
        m_manual = manual;
        m_maker = maker;
    }

    @Override
    public T apply(T position_M) {
        // return new CrankWorkstate(m_manual.getAsDouble());
        return m_maker.apply(m_manual.getAsDouble());
    }

    @Override
    public ProfileFollower<T> withProfile(MotionProfile profile) {
        // ignore the profile
        return this;
    }
}
