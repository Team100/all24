package org.team100.lib.motion.example1d;

import org.team100.lib.profile.MotionProfile;

import edu.wpi.first.wpilibj.Timer;

/**
 * Follows the given profile with feed forward only,
 * so it just blindly follows the profile.
 */
public class FFVelocitySupplier1d implements ProfileFollower {
    private final Timer m_timer;
    private MotionProfile m_profile;

    /**
     * Supplies zero until a profile is specified. Instantiate once per command
     * invocation, don't reuse it.
     */
    public FFVelocitySupplier1d() {
        m_timer = new Timer();
        // there is no safe default profile.
        m_profile = null;
    }

    @Override
    public void accept(MotionProfile profile) {
        m_profile = profile;
        m_timer.restart();
    }

    /** @return velocity in meters per second */
    @Override
    public Double apply(double position_M) {
        if (m_profile == null)
            return 0.0;
        return m_profile.get(m_timer.get()).getV();
    }
}
