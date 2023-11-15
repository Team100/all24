package org.team100.lib.motion.example1d.crank;

import java.util.function.Supplier;

import org.team100.lib.profile.MotionProfile;

import edu.wpi.first.wpilibj.Timer;

/**
 * Follows the given profile with feed forward only,
 * so it just blindly follows the profile.
 * TODO: ignore the supplied measurement
 */
public class CrankFFVelocitySupplier1d implements CrankProfileFollower {
    private final Timer m_timer;
    private final MotionProfile m_profile;
    private final Supplier<CrankWorkstate> m_measurement;

    /**
     * Supplies zero until a profile is specified. Instantiate once per command
     * invocation, don't reuse it.
     */
    public CrankFFVelocitySupplier1d(Supplier<CrankWorkstate> measurement) {
        this(null, measurement);
    }

    /**
     * Return a new instance with the specified profile.
     */
    @Override
    public CrankFFVelocitySupplier1d withProfile(MotionProfile profile) {
        return new CrankFFVelocitySupplier1d(profile, m_measurement);
    }

    // TODO: this seems wrong
    @Override
    public CrankWorkstate get() {
        if (m_profile == null)
            return m_measurement.get();
        // this is wrong; return position or velocity here? or both?
        return new CrankWorkstate(m_profile.get(m_timer.get()).getV());
    }

    private CrankFFVelocitySupplier1d(MotionProfile profile, Supplier<CrankWorkstate> measurement) {
        m_timer = new Timer();
        m_profile = profile;
        m_measurement = measurement;
    }

}
