package org.team100.lib.motion.example1d;

import java.util.function.DoubleFunction;

import org.team100.lib.motion.example1d.framework.WorkspaceController;
import org.team100.lib.motion.example1d.framework.Workstate;
import org.team100.lib.profile.MotionProfile;
import org.team100.lib.profile.MotionState;

import edu.wpi.first.wpilibj.Timer;

/**
 * Follows the given profile with a PIDF controller.
 * 
 * TODO: split the profile-following part from the PID-controlling part.
 */
public class PIDVelocitySupplier1d<T extends Workstate<T>> implements ProfileFollower<T> {
    private final WorkspaceController<T> m_controller;
    private final Timer m_timer;
    private final MotionProfile m_profile;
    private final DoubleFunction<T> m_maker;

    /**
     * Supplies zero until a profile is specified. Instantiate once per command
     * invocation, don't reuse it.
     */
    public PIDVelocitySupplier1d(WorkspaceController<T> controller, DoubleFunction<T> maker) {
        this(controller, maker, null);
    }

    @Override
    public T apply(T position_M) {
        if (m_profile == null)
            return m_maker.apply(0.0);
        // TODO: wrap the profile in the same type to avoid wrapping here.
        MotionState motionState = m_profile.get(m_timer.get());
        return m_controller.calculate(position_M.getWorkstate(), motionState);
    }

    @Override
    public ProfileFollower<T> withProfile(MotionProfile profile) {
        return new PIDVelocitySupplier1d<>(m_controller, m_maker, profile);
    }

    private PIDVelocitySupplier1d(
            WorkspaceController<T> controller,
            DoubleFunction<T> maker,
            MotionProfile profile) {
        m_controller = controller;
        m_maker = maker;
        // m_controller = new PIDController(1, 0, 0);
        m_timer = new Timer();
        m_profile = profile;
    }

}
