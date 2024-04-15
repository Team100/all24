package org.team100.lib.util;

import static org.junit.jupiter.api.Assertions.assertEquals;

import org.junit.jupiter.api.Test;

class ExpiringMemoizingSupplierTest {
    private int value = 0;

    @Test
    void testSimple() {
        ExpiringMemoizingSupplier<String> s = new ExpiringMemoizingSupplier<>(
                () -> "hi", 10000);
        assertEquals("hi", s.get());
    }

    @Test
    void testTiming() throws InterruptedException {
        ExpiringMemoizingSupplier<Integer> s = new ExpiringMemoizingSupplier<>(
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
