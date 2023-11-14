package org.team100.lib.motion.example1d;

import java.util.function.DoubleFunction;

import org.team100.lib.motion.example1d.framework.Workstate;
import org.team100.lib.profile.MotionProfile;

import edu.wpi.first.wpilibj.Timer;

/**
 * Follows the given profile with feed forward only,
 * so it just blindly follows the profile.
 */
public class FFVelocitySupplier1d<T extends Workstate<T>> implements ProfileFollower<T> {
    private final Timer m_timer;
    private final MotionProfile m_profile;
    private final DoubleFunction<T> m_maker;

    /**
     * Supplies zero until a profile is specified. Instantiate once per command
     * invocation, don't reuse it.
     */
    public FFVelocitySupplier1d(DoubleFunction<T> maker) {
        this(maker, null);
    }

    /**
     * Return a new instance with the specified profile.
     */
    @Override
    public FFVelocitySupplier1d<T> withProfile(MotionProfile profile) {
        return new FFVelocitySupplier1d<>(m_maker, profile);
    }

    // TODO: this seems wrong
    @Override
    public T apply(T position_M) {
        if (m_profile == null)
            return position_M;
        // this is wrong; return position or velocity here? or both?
        // return new CrankWorkstate(m_profile.get(m_timer.get()).getV());
        return m_maker.apply(m_profile.get(m_timer.get()).getV());
    }

    private FFVelocitySupplier1d(DoubleFunction<T> maker, MotionProfile profile) {
        m_maker = maker;
        m_timer = new Timer();
        m_profile = profile;
    }

}
