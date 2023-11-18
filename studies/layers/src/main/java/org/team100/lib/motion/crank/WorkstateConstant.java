package org.team100.lib.motion.crank;

public class WorkstateConstant implements Workstates {
    private final Workstate m_state;

    public WorkstateConstant(Workstate state) {
        m_state = state;
    }

    @Override
    public Workstate get() {
        return m_state;
    }

    @Override
    public void accept(Indicator indicator) {
        indicator.indicate(this);
    }
}
