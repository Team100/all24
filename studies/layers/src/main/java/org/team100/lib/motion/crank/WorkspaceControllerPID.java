package org.team100.lib.motion.crank;

import java.util.function.Supplier;

import org.team100.lib.profile.MotionProfile;
import org.team100.lib.profile.MotionState;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Timer;

/**
 * Follows the given profile with a PIDF controller.
 * 
 * TODO: split the profile-following part from the PID-controlling part.
 */
public class WorkspaceControllerPID implements Workstates {
    private final Timer m_timer;
    private final Supplier<Supplier<MotionProfile>> m_profile;
    private final Supplier<Workstates> m_measurement;
    private final PIDController m_controller;

    public WorkspaceControllerPID(
            Supplier<Supplier<MotionProfile>> profile,
            Supplier<Workstates> measurement) {
        m_timer = new Timer();
        m_profile = profile;
        m_measurement = measurement;
        m_controller = new PIDController(1,0,0);
    }

    @Override
    public Workstate get() {
        if (m_profile.get().get() == null)
            return m_measurement.get().get();

        // TODO: wrap the profile in the same type to avoid wrapping here.
        MotionState reference = m_profile.get().get().get(m_timer.get());
        Workstate measurement = m_measurement.get().get().getWorkstate();

        double u_FF = reference.getV();
        double u_FB = m_controller.calculate(measurement.getState(), reference.getX());

        return new Workstate(u_FF + u_FB);
    }

    @Override
    public void accept(Indicator indicator) {
        m_measurement.get().accept(indicator);
        indicator.indicate(this);
    }
}
