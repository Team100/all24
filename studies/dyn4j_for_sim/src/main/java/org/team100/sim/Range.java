package org.team100.sim;

/**
 * Represents the vertical extent of an object.
 * 
 * Used to filter collisions.
 */
public class Range {
    private final double m_min;
    private final double m_max;

    public Range(double min, double max) {
        if (max <= min)
            throw new IllegalArgumentException("max must be greater than min");
        m_min = min;
        m_max = max;
    }

    public boolean overlaps(Range other) {
        return  m_min <= other.m_max && m_max >= other.m_min;
    }

}