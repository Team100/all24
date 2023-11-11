package org.team100.two_joint_arm.subsystems.arm;

import org.team100.lib.controller.State100;

// TODO: do something with this
public class ArmState {
    private final State100 m_lower;
    private final State100 m_upper;

    public ArmState(State100 lower, State100 upper) {
        m_lower = lower;
        m_upper = upper;
    }

    public State100 lower() {
        return m_lower;
    }

    public State100 upper() {
        return m_upper;
    }
}