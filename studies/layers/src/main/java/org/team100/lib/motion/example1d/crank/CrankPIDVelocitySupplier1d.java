package org.team100.lib.motion.example1d.crank;

import java.util.function.Supplier;

import org.team100.lib.profile.MotionProfile;
import org.team100.lib.profile.MotionState;

import edu.wpi.first.wpilibj.Timer;

/**
 * Follows the given profile with a PIDF controller.
 * 
 * TODO: split the profile-following part from the PID-controlling part.
 */
public class CrankPIDVelocitySupplier1d implements Supplier<CrankWorkstate> {
    private final CrankWorkspaceController m_controller;
    private final Timer m_timer;
    private final Supplier<Supplier<MotionProfile>> m_profile;
    private final Supplier<CrankWorkstate> m_measurement;

    public CrankPIDVelocitySupplier1d(
            CrankWorkspaceController controller,
            Supplier<Supplier<MotionProfile>> profile,
            Supplier<CrankWorkstate> measurement) {
        m_controller = controller;
        m_timer = new Timer();
        m_profile = profile;
        m_measurement = measurement;
    }

    @Override
    public CrankWorkstate get() {
        if (m_profile.get().get() == null)
            return m_measurement.get();
        // TODO: wrap the profile in the same type to avoid wrapping here.
        MotionState motionState = m_profile.get().get().get(m_timer.get());
        return m_controller.calculate(m_measurement.get().getWorkstate(), motionState);
    }
}
