package org.team100.lib.motion.crank;

import java.util.function.Supplier;

/** Kinematics that works in the DAG. */
public class InverseKinematics implements Configurations {
    private final Supplier<Workstates> m_sliderPosition;
    private final Kinematics m_kinematics;

    public InverseKinematics(
            Supplier<Workstates> sliderPosition,
            Kinematics kinematics) {
        m_sliderPosition = sliderPosition;
        m_kinematics = kinematics;
    }

    @Override
    public Configuration get() {
        return m_kinematics.inverse(m_sliderPosition.get().get());
    }

    @Override
    public void accept(Indicator indicator) {
        m_sliderPosition.get().accept(indicator);
        indicator.indicate(this);
    }

}
