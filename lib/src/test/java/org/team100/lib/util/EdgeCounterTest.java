package org.team100.lib.util;

import static org.junit.jupiter.api.Assertions.assertEquals;

import org.junit.jupiter.api.Test;

class EdgeCounterTest {

    @Test
    void testCounter() {
        EdgeCounter c = new EdgeCounter(0.25, 0.75);
        assertEquals(0, c.update(0));
        assertEquals(0, c.update(0.1));
        assertEquals(0, c.update(0.5));
        assertEquals(0, c.update(0.9));
        assertEquals(1, c.update(0.1));
        assertEquals(1, c.update(0.5));
        assertEquals(1, c.update(0.1));
        assertEquals(0, c.update(0.9));
    }
}
