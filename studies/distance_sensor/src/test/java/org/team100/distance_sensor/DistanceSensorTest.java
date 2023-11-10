package org.team100.distance_sensor;

import static org.junit.jupiter.api.Assertions.assertEquals;

import org.junit.jupiter.api.Test;
import org.team100.distance_sensor.subsystems.distance_sensor.DistanceSensor;
import org.team100.distance_sensor.subsystems.distance_sensor.NTDistanceSensor;

class DistanceSensorTest {
    private static final double kDelta = 0.001;

    @Test
    void testSensor() {
        DistanceSensor s = new NTDistanceSensor("foo");
        s.periodic();
        // -2 is the default
        assertEquals(-2, s.getCentimeters(), kDelta);
    }
}
