package org.team100.lib.motion.example1d;

import java.util.function.DoubleSupplier;

import org.team100.lib.profile.MotionProfile;

import edu.wpi.first.wpilibj.Timer;

/**
 * Follows the given profile with feed forward only,
 * so it just blindly follows the profile.
 * 
 * the timer starts immediately upon construction.
 */
public class FFVelocitySupplier1d implements DoubleSupplier {
    private final MotionProfile m_profile;
    private final Timer m_timer;

    public FFVelocitySupplier1d(MotionProfile profile) {
        m_profile = profile;
        m_timer = new Timer();
        m_timer.start();
    }

    /** @return velocity in meters per second */
    @Override
    public double getAsDouble() {
        return m_profile.get(m_timer.get()).getV();
    }
}
