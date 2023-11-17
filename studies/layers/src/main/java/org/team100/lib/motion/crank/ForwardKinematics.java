package org.team100.lib.motion.crank;

import java.util.function.Supplier;

public class ForwardKinematics implements Workstates {
    private final Supplier<Configurations> m_crankAngleRad;
    private final Kinematics m_kinematics;

    public ForwardKinematics(
            Supplier<Configurations> crankAngleRad,
            Kinematics kinematics) {
        m_crankAngleRad = crankAngleRad;
        m_kinematics = kinematics;
    }

    @Override
    public Workstate get() {
        return m_kinematics.forward(m_crankAngleRad.get().get());
    }

    @Override
    public void accept(Indicator indicator) {
        m_crankAngleRad.get().accept(indicator);
        indicator.indicate(this);
    }
}
