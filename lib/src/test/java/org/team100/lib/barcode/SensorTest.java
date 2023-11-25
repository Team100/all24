package org.team100.lib.barcode;

import static org.junit.jupiter.api.Assertions.assertEquals;

import org.junit.jupiter.api.Test;

public class SensorTest {

    @Test
    void testToChannel() {
        assertEquals(0, Sensor.toChannel(new boolean[] { false, false, false, false }));
        assertEquals(1, Sensor.toChannel(new boolean[] { true, false, false, false }));
        assertEquals(2, Sensor.toChannel(new boolean[] { false, true, false, false }));
        assertEquals(3, Sensor.toChannel(new boolean[] { true, true, false, false }));
        assertEquals(4, Sensor.toChannel(new boolean[] { false, false, true, false }));
        assertEquals(5, Sensor.toChannel(new boolean[] { true, false, true, false }));
        assertEquals(6, Sensor.toChannel(new boolean[] { false, true, true, false }));
        assertEquals(7, Sensor.toChannel(new boolean[] { true, true, true, false }));
        assertEquals(8, Sensor.toChannel(new boolean[] { false, false, false, true }));
        assertEquals(9, Sensor.toChannel(new boolean[] { true, false, false, true }));
        assertEquals(10, Sensor.toChannel(new boolean[] { false, true, false, true }));
        assertEquals(11, Sensor.toChannel(new boolean[] { true, true, false, true }));
        assertEquals(12, Sensor.toChannel(new boolean[] { false, false, true, true }));
        assertEquals(13, Sensor.toChannel(new boolean[] { true, false, true, true }));
        assertEquals(14, Sensor.toChannel(new boolean[] { false, true, true, true }));
        assertEquals(15, Sensor.toChannel(new boolean[] { true, true, true, true }));
    }
}
