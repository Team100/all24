package org.team100.lib.motion.example1d;

import org.team100.lib.motion.example1d.framework.Workstate;
import org.team100.lib.profile.MotionProfile;
import org.team100.lib.profile.MotionState;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Timer;

/**
 * Follows the given profile with a PIDF controller.
 */
public class PIDVelocitySupplier1d implements ProfileFollower {
    private final PIDController m_controller;
    private final Timer m_timer;
    private final MotionProfile m_profile;

    /**
     * Supplies zero until a profile is specified. Instantiate once per command
     * invocation, don't reuse it.
     */
    public PIDVelocitySupplier1d() {
        this(null);
    }

    @Override
    public Workstate<Double> apply(Workstate<Double> position_M) {
        if (m_profile == null)
            return new CrankWorkstate(0.0);
        MotionState motionState = m_profile.get(m_timer.get());
        double u_FF = motionState.getV();
        double u_FB = m_controller.calculate(position_M.getWorkstate(), motionState.getX());
        return new CrankWorkstate(u_FF + u_FB);
    }

    @Override
    public ProfileFollower withProfile(MotionProfile profile) {
        return new PIDVelocitySupplier1d(profile);
    }

    private PIDVelocitySupplier1d(MotionProfile profile) {
        m_controller = new PIDController(1, 0, 0);
        m_timer = new Timer();
        m_profile = profile;
    }

}
