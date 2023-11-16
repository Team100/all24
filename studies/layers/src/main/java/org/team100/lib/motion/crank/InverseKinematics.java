package org.team100.lib.motion.crank;

import java.util.function.Supplier;

/** Kinematics that works in the DAG. */
public class InverseKinematics implements Supplier<Configuration> {
    private final Supplier<Workstate> m_sliderPosition;
    private final Kinematics m_kinematics;

    public InverseKinematics(
            Supplier<Workstate> sliderPosition,
            Kinematics kinematics) {
        m_sliderPosition = sliderPosition;
        m_kinematics = kinematics;
    }

    @Override
    public Configuration get() {
        return m_kinematics.inverse(m_sliderPosition.get());
    }

}
