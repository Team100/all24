package org.team100.lib.motion.crank;

import java.util.function.Supplier;

import org.team100.lib.profile.MotionProfile;

import edu.wpi.first.wpilibj.Timer;

/**
 * Follows the given profile with feed forward only,
 * so it just blindly follows the profile.
 * TODO: ignore the supplied measurement
 */
public class WorkspaceControllerFF implements Supplier<Workstate> {
    private final Timer m_timer;
    private final Supplier<Supplier<MotionProfile>> m_profile;
    private final Supplier<Workstate> m_measurement;

    public WorkspaceControllerFF(
            Supplier<Supplier<MotionProfile>> profile,
            Supplier<Workstate> measurement) {
        m_timer = new Timer();
        m_profile = profile;
        m_measurement = measurement;
    }

    // TODO: this seems wrong
    @Override
    public Workstate get() {
        if (m_profile.get().get() == null)
            return m_measurement.get();
        // this is wrong; return position or velocity here? or both?
        return new Workstate(m_profile.get().get().get(m_timer.get()).getV());
    }
}
