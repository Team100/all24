package org.team100.lib.util;

import static org.junit.jupiter.api.Assertions.assertEquals;

import org.junit.jupiter.api.Test;

class TimedCacheTest {
    private int value = 0;

    @Test
    void testSimple() {
        TimedCache<String> s = new TimedCache<>(
                () -> "hi", 10000);
        assertEquals("hi", s.get());
    }

    @Test
    void testTiming() throws InterruptedException {
        TimedCache<Integer> s = new TimedCache<>(
                () -> ++value, 10000);
        // first call increments
        assertEquals(1, s.get());
        // second call is memoized
        assertEquals(1, s.get());
        Thread.sleep(10);
        // after awhile, it expires the old result
        assertEquals(2, s.get());
        // and remembers the new value
        assertEquals(2, s.get());
    }
}
