package org.team100.lib.barcode;

import static org.junit.jupiter.api.Assertions.assertArrayEquals;

import org.junit.jupiter.api.Test;

public class MuxTest {
    
    @Test
    void testToArray() {
        assertArrayEquals(new boolean[] { false, false, false, false }, Mux.toArray(0, 4));
        assertArrayEquals(new boolean[] { true, false, false, false }, Mux.toArray(1, 4));
        assertArrayEquals(new boolean[] { false, true, false, false }, Mux.toArray(2, 4));
        assertArrayEquals(new boolean[] { true, true, false, false }, Mux.toArray(3, 4));
        assertArrayEquals(new boolean[] { false, false, true, false }, Mux.toArray(4, 4));
        assertArrayEquals(new boolean[] { true, false, true, false }, Mux.toArray(5, 4));
        assertArrayEquals(new boolean[] { false, true, true, false }, Mux.toArray(6, 4));
        assertArrayEquals(new boolean[] { true, true, true, false }, Mux.toArray(7, 4));
        assertArrayEquals(new boolean[] { false, false, false, true }, Mux.toArray(8, 4));
        assertArrayEquals(new boolean[] { true, false, false, true }, Mux.toArray(9, 4));
        assertArrayEquals(new boolean[] { false, true, false, true }, Mux.toArray(10, 4));
        assertArrayEquals(new boolean[] { true, true, false, true }, Mux.toArray(11, 4));
        assertArrayEquals(new boolean[] { false, false, true, true }, Mux.toArray(12, 4));
        assertArrayEquals(new boolean[] { true, false, true, true }, Mux.toArray(13, 4));
        assertArrayEquals(new boolean[] { false, true, true, true }, Mux.toArray(14, 4));
        assertArrayEquals(new boolean[] { true, true, true, true }, Mux.toArray(15, 4));
    }
}
