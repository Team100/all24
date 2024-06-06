package org.team100.util;

import static org.junit.jupiter.api.Assertions.assertEquals;

import org.junit.jupiter.api.Test;

class LatchTest {
    boolean value = false;

    @Test
    void testSimple() {
        Latch l = new Latch(()->value);
        l.periodic();
        assertEquals(false, l.getAsBoolean());
        l.periodic();
        assertEquals(false, l.getAsBoolean());
        value = true;
        l.periodic();
        assertEquals(true, l.getAsBoolean());
        assertEquals(false, l.getAsBoolean());
        l.periodic();
        assertEquals(false, l.getAsBoolean());
        value = false;
        l.periodic();
        assertEquals(false, l.getAsBoolean());
        assertEquals(false, l.getAsBoolean());
        value = true;
        l.periodic();
        assertEquals(true, l.getAsBoolean());
        assertEquals(false, l.getAsBoolean());
        value = true;
        l.reset();
        assertEquals(false, l.getAsBoolean());
    }

}
