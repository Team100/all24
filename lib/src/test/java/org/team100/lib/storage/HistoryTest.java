package org.team100.lib.storage;

import static org.junit.jupiter.api.Assertions.assertEquals;

import java.util.Map.Entry;

import org.junit.jupiter.api.Test;

class HistoryTest {
    @Test
    void testSimple() {
        History<String> h = new History<>(100);
        h.put(1.0, "hi 1");
        h.put(0.5, "hi 0.5");
        Entry<Double, String> e = h.validFloorEntry(2.0);
        assertEquals(1.0, e.getKey());
        assertEquals("hi 1", e.getValue());
    }
}
