package org.team100.lib.motion.example1d;

import java.util.function.DoubleFunction;

import org.team100.lib.motion.example1d.framework.Workstate;
import org.team100.lib.profile.MotionProfile;

/**
 * Always supplies zero, for "safe mode."
 */
public class ZeroVelocitySupplier1d<T extends Workstate<T>> implements ProfileFollower<T> {

    private final DoubleFunction<T> m_maker;

    public ZeroVelocitySupplier1d(DoubleFunction<T> maker) {
        m_maker = maker;
    }

    @Override
    public T apply(T position_M) {
        // return new CrankWorkstate(0.0);
        return m_maker.apply(0.0);
    }

    @Override
    public ProfileFollower<T> withProfile(MotionProfile profile) {
        // ignore the profile
        return this;
    }
}
