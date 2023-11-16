package org.team100.lib.motion.crank;

import java.util.function.Supplier;

public class ForwardKinematics implements Supplier<Workstate> {
    private final Supplier<Configuration> m_crankAngleRad;
    private final Kinematics m_kinematics;

    public ForwardKinematics(
            Supplier<Configuration> crankAngleRad,
            Kinematics kinematics) {
        m_crankAngleRad = crankAngleRad;
        m_kinematics = kinematics;
    }

    @Override
    public Workstate get() {
        return m_kinematics.forward(m_crankAngleRad.get());
    }
}
