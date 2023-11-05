package org.team100.lib.util;

/**
 * Observe an analog signal: count up on negative edges, count down
 * on positive edges. Use this to unroll angular sensors.
 */
public class EdgeCounter {
    private final double m_lo;
    private final double m_hi;
    private double m_value;
    private int m_count;

    public EdgeCounter(double lo, double hi) {
        m_lo = lo;
        m_hi = hi;
    }

    public int update(double value) {
        if (m_value > m_hi && value < m_lo)
            ++m_count;
        else if (m_value < m_lo && value > m_hi)
            --m_count;
        m_value = value;
        return m_count;
    }

}
