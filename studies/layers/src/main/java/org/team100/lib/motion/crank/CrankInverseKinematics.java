package org.team100.lib.motion.crank;

import java.util.function.Supplier;

/** Kinematics that works in the DAG. */
public class CrankInverseKinematics implements Supplier<CrankConfiguration> {
    private final Supplier<CrankWorkstate> m_sliderPosition;
    private final CrankKinematics m_kinematics;

    public CrankInverseKinematics(
            Supplier<CrankWorkstate> sliderPosition,
            CrankKinematics kinematics) {
        m_sliderPosition = sliderPosition;
        m_kinematics = kinematics;
    }

    @Override
    public CrankConfiguration get() {
        return m_kinematics.inverse(m_sliderPosition.get());
    }

}
