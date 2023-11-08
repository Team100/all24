package org.team100.lib.motion.example1d;

import java.util.function.DoublePredicate;

public class PositionLimit implements DoublePredicate {
    private final double m_min;
    private final double m_max;

    public PositionLimit(double min, double max) {
        m_min = min;
        m_max = max;
    }

    @Override
    public boolean test(double position_M) {
        if (position_M < m_min)
            return false;
        if (position_M > m_max)
            return false;
        return true;
    }

}
