package org.team100.lib.motion.crank;

import java.util.function.Supplier;

public class CrankForwardKinematics implements Supplier<CrankWorkstate> {
    private final Supplier<CrankConfiguration> m_crankAngleRad;
    private final CrankKinematics m_kinematics;

    public CrankForwardKinematics(
            Supplier<CrankConfiguration> crankAngleRad,
            CrankKinematics kinematics) {
        m_crankAngleRad = crankAngleRad;
        m_kinematics = kinematics;
    }

    @Override
    public CrankWorkstate get() {
        return m_kinematics.forward(m_crankAngleRad.get());
    }
}
