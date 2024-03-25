package org.team100.lib.indicator;

/** Represents a segment of an LED buffer: a range with a length and offset. */
public class LEDStrip {
    private final int m_length;
    private final int m_offset;
    public LEDStrip(int length, int offset) {
        m_length = length;
        m_offset = offset;
    }

    public int getLength() {
        return m_length;
    }

    public int getOffset() {
        return m_offset;
    }
}
