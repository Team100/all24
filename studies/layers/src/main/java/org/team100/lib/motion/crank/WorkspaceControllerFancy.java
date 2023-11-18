package org.team100.lib.motion.crank;

import java.util.function.Supplier;

/**
 * Illustrates monolithic sampling and control, using an arbitrary path
 * specification and arbitrary control logic.
 */
public class WorkspaceControllerFancy implements Workstates {
    public static class Spec {
        public double x = 0;
    }
    public static class Specs implements Supplier<Spec> {
        @Override
        public Spec get() {
            return new Spec();
        }
    }

    private final Supplier<Supplier<Spec>> m_profile;
    private final Supplier<Workstates> m_measurement;

    public WorkspaceControllerFancy(
            Supplier<Supplier<Spec>> profile,
            Supplier<Workstates> measurement) {
        m_profile = profile;
        m_measurement = measurement;
    }

    @Override
    public Workstate get() {
        if (m_profile.get().get() == null)
            return null;
        // illustrates that *any* logic can be used to generate workspace references.
        if (m_measurement.get().get().getWorkstate().getState() < m_profile.get().get().x)
            return new Workstate(1.0);
        return new Workstate(-1.0);

    }

    @Override
    public void accept(Indicator indicator) {
        m_measurement.get().accept(indicator);
        indicator.indicate(this);
    }
}
