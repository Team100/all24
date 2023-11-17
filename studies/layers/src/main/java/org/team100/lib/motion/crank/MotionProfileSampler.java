package org.team100.lib.motion.crank;

import java.util.function.Supplier;

import org.team100.lib.profile.MotionState;

import edu.wpi.first.wpilibj.Timer;

/** Samples a profile in workspace. */
public class MotionProfileSampler implements Workstates {
    private final Timer m_timer;
    private final Supplier<MotionProfiles> m_profile;

    /** Starts the timer immediately. */
    public MotionProfileSampler(Supplier<MotionProfiles> profile) {
        m_timer = new Timer();
        m_profile = profile;
        m_timer.start();
    }

    @Override
    public Workstate get() {
        if (m_profile.get().get() == null)
            return null;
        MotionState reference = m_profile.get().get().get(m_timer.get());
        // TODO: do this wrapping some other way
        return new Workstate(reference.getX());
    }

    @Override
    public void accept(Indicator indicator) {
        m_profile.get().accept(indicator);
        indicator.indicate(this);
    }

}
