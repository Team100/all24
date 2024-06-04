package org.team100.util;

import static org.junit.jupiter.api.Assertions.assertEquals;

import org.junit.jupiter.api.Test;

class CounterTest {
    boolean value = false;

    @Test
    void testSimple() {
        Counter c = new Counter(() -> value);
        assertEquals(0, c.getAsInt());
        assertEquals(0, c.getAsInt());
        value = true;
        assertEquals(1, c.getAsInt());
        assertEquals(1, c.getAsInt());
        value = false;
        assertEquals(1, c.getAsInt());
        assertEquals(1, c.getAsInt());
        value = true;
        assertEquals(2, c.getAsInt());
        assertEquals(2, c.getAsInt());
        c.reset();
        assertEquals(0, c.getAsInt());
        assertEquals(0, c.getAsInt());
    }
}
