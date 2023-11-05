package org.team100.lib.util;

import static org.junit.jupiter.api.Assertions.assertEquals;

// passes with this uncommented
// import com.acmerobotics.roadrunner.util.DoubleProgression;

import org.junit.jupiter.api.Test;

class DoubleProgressionTest {
    private static final double kDelta = 0.001;

    @Test
    void testBasic() {
        DoubleProgression p = new DoubleProgression(1, 0.1, 10);
        assertEquals(10, p.size());
        assertEquals(1.5, p.get(5), kDelta);
    }

    @Test
    void testFromClosedInterval() {
        DoubleProgression p =  DoubleProgression.fromClosedInterval(1, 2, 9);
        assertEquals(9, p.size());
        assertEquals(1.5, p.get(4), kDelta);
    }
}
