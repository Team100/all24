package org.team100.lib.motion.crank;

import java.util.function.Supplier;

import edu.wpi.first.math.controller.PIDController;

/**
 * Follows the given profile with a PIDF controller.
 */
public class WorkspaceControllerPID implements Workstates {
    private final Supplier<Workstates> m_reference;
    private final Supplier<Workstates> m_measurement;
    private final PIDController m_controller;

    /** Illustrates decoupling the sampler from the controller. */
    public WorkspaceControllerPID(Supplier<Workstates> reference, Supplier<Workstates> measurement) {
        m_reference = reference;
        m_measurement = measurement;
        m_controller = new PIDController(1, 0, 0);
    }

    @Override
    public Workstate get() {
        if (m_reference.get().get() == null)
            return m_measurement.get().get();

        Workstate reference = m_reference.get().get();
        Workstate measurement = m_measurement.get().get().getWorkstate();

        double u_FF = reference.getState();
        double u_FB = m_controller.calculate(measurement.getState(), reference.getState());

        return new Workstate(u_FF + u_FB);
    }

    @Override
    public void accept(Indicator indicator) {
        m_reference.get().accept(indicator);
        m_measurement.get().accept(indicator);
        indicator.indicate(this);
    }
}
