package org.team100.lib.motion.example1d;

import org.team100.lib.motion.example1d.framework.Workstate;
import org.team100.lib.profile.MotionProfile;

import edu.wpi.first.wpilibj.Timer;

/**
 * Follows the given profile with feed forward only,
 * so it just blindly follows the profile.
 */
public class FFVelocitySupplier1d implements ProfileFollower {
    private final Timer m_timer;
    private final MotionProfile m_profile;

    /**
     * Supplies zero until a profile is specified. Instantiate once per command
     * invocation, don't reuse it.
     */
    public FFVelocitySupplier1d() {
        this(null);
    }

    /**
     * Return a new instance with the specified profile.
     */
    @Override
    public FFVelocitySupplier1d withProfile(MotionProfile profile) {
        return new FFVelocitySupplier1d(profile);
    }

    // TODO: this seems wrong
    @Override
    public Workstate<Double> apply(Workstate<Double> position_M) {
        if (m_profile == null)
            return position_M;
        // this is wrong; return position or velocity here? or both?
        return new CrankWorkstate(m_profile.get(m_timer.get()).getV());
    }

    private FFVelocitySupplier1d(MotionProfile profile) {
        m_timer = new Timer();
        m_profile = profile;
    }

}
