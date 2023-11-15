package org.team100.lib.motion.example1d.crank;

import org.team100.lib.profile.MotionProfile;
import org.team100.lib.profile.MotionState;

import edu.wpi.first.wpilibj.Timer;

/**
 * Follows the given profile with a PIDF controller.
 * 
 * TODO: split the profile-following part from the PID-controlling part.
 */
public class CrankPIDVelocitySupplier1d implements CrankProfileFollower {
    private final CrankWorkspaceController m_controller;
    private final Timer m_timer;
    private final MotionProfile m_profile;

    /**
     * Supplies zero until a profile is specified. Instantiate once per command
     * invocation, don't reuse it.
     */
    public CrankPIDVelocitySupplier1d(CrankWorkspaceController controller) {
        this(controller, null);
    }

    @Override
    public CrankWorkstate apply(CrankWorkstate position_M) {
        if (m_profile == null)
            return new CrankWorkstate(0.0);
        // TODO: wrap the profile in the same type to avoid wrapping here.
        MotionState motionState = m_profile.get(m_timer.get());
        return m_controller.calculate(position_M.getWorkstate(), motionState);
    }

    @Override
    public CrankProfileFollower withProfile(MotionProfile profile) {
        return new CrankPIDVelocitySupplier1d(m_controller, profile);
    }

    private CrankPIDVelocitySupplier1d(
            CrankWorkspaceController controller,
            MotionProfile profile) {
        m_controller = controller;
        // m_controller = new PIDController(1, 0, 0);
        m_timer = new Timer();
        m_profile = profile;
    }

}
