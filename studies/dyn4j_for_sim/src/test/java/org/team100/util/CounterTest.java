package org.team100.util;

import static org.junit.jupiter.api.Assertions.assertEquals;

import org.junit.jupiter.api.Test;

class CounterTest {
    boolean value = false;

    @Test
    void testSimple() {
        Counter c = new Counter(() -> value);
        c.periodic();
        assertEquals(0, c.getAsInt());
        c.periodic();
        assertEquals(0, c.getAsInt());
        c.periodic();
        value = true;
        c.periodic();
        assertEquals(1, c.getAsInt());
        c.periodic();
        assertEquals(1, c.getAsInt());
        c.periodic();
        value = false;
        c.periodic();
        assertEquals(1, c.getAsInt());
        c.periodic();
        assertEquals(1, c.getAsInt());
        c.periodic();
        value = true;
        c.periodic();
        assertEquals(2, c.getAsInt());
        c.periodic();
        assertEquals(2, c.getAsInt());
        c.periodic();
        c.reset();
        c.periodic();
        assertEquals(0, c.getAsInt());
        c.periodic();
        assertEquals(0, c.getAsInt());
        c.periodic();
    }
}
