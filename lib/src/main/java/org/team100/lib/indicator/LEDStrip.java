package org.team100.lib.indicator;

import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.util.Color;

/**
 * Represents a segment of an LED buffer: a range with a start (inclusive) and
 * end (exclusive).
 */
public class LEDStrip {
    private final int m_start;
    private final int m_end;

    public LEDStrip(int start, int end) {
        m_start = start;
        m_end = end;
    }

    /**
     * Fill the appropriate section of the buffer.
     */
    public void solid(AddressableLEDBuffer buffer, Color color) {
        for (int i = start(); i < end(); i++) {
            buffer.setLED(i, color);
        }
    }

    /** First led in the range. */
    int start() {
        return m_start;
    }

    /** End of the range, plus one */
    int end() {
        return m_end;
    }
}
