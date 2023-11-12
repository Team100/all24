package org.team100.lib.motion.example1d;

import org.team100.lib.motion.example1d.framework.Workstate;

public class CrankWorkstate implements Workstate<Double> {

    private final Double m_state;

    public CrankWorkstate(Double state) {
        m_state = state;
    }

    @Override
    public Double getWorkstate() {
        return m_state;
    }

}
