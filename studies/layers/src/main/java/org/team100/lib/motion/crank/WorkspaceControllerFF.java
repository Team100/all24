package org.team100.lib.motion.crank;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj.Timer;

/**
 * Follows the given profile with feed forward only, ignoring the measurement,
 * so it just blindly follows the profile.
 */
public class WorkspaceControllerFF implements Workstates {
    private final Timer m_timer;
    private final Supplier<MotionProfiles> m_profile;

    /** Illustrates sampling inside the controller. */
    public WorkspaceControllerFF(Supplier<MotionProfiles> profile) {
        m_timer = new Timer();
        m_profile = profile;
        m_timer.start();
    }

    @Override
    public Workstate get() {
        if (m_profile.get().get() == null)
            return null;
        // this is wrong; return position or velocity here? or both?
        return new Workstate(m_profile.get().get().get(m_timer.get()).getV());
    }

    @Override
    public void accept(Indicator indicator) {
        indicator.indicate(this);
    }
}
