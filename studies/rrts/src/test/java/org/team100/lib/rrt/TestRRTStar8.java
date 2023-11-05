package org.team100.lib.rrt;

import static org.junit.jupiter.api.Assertions.assertEquals;

import org.junit.jupiter.api.Test;

public class TestRRTStar8 {
    @Test
    void testPLP() {
        // these NaN's mean that no PLP is possible, it's PP.
        assertEquals(2.5, RRTStar8.aPLP(0, 0, 1, 1, 1, 2), 0.001);
        assertEquals(Double.NaN, RRTStar8.CalcTotalTime(0, 0, 1, 1, 2.5, 2), 0.001);
        assertEquals(0.8, RRTStar8.CalcSwitchTime1(0, 0, 1, 1, 2.5, 2), 0.001);
        assertEquals(Double.NaN, RRTStar8.CalcSwitchTime2(0, 0, 1, 1, 2.5, 2), 0.001);

        // try points further apart, i did this by hand
        assertEquals(2, RRTStar8.aPLP(0, 0, 5, 0, 3.5, 2), 0.001);
        assertEquals(3.5, RRTStar8.CalcTotalTime(0, 0, 5, 0, 2, 2), 0.001);
        assertEquals(1.0, RRTStar8.CalcSwitchTime1(0, 0, 5, 0, 2, 2), 0.001);
        assertEquals(2.5, RRTStar8.CalcSwitchTime2(0, 0, 5, 0, 2, 2), 0.001);

    }
}
