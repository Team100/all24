package org.team100.lib.motion.example1d.crank;

import org.team100.lib.motion.example1d.framework.WorkspaceController;
import org.team100.lib.profile.MotionState;

import edu.wpi.first.math.controller.PIDController;

/** TODO: make this operate on full state. */
public class CrankWorkspaceController implements WorkspaceController<CrankWorkstate> {
    private final PIDController m_controller;

    public CrankWorkspaceController() {
        m_controller = new PIDController(1,0,0);
    }

    // this mixes two ways to describe state; clean it up.
    @Override
    public CrankWorkstate calculate(CrankWorkstate measurement, MotionState ref) {
        double u_FF = ref.getV();
        double u_FB = m_controller.calculate(measurement.getState(), ref.getX());
        return new CrankWorkstate(u_FF + u_FB);
    }
    
}
