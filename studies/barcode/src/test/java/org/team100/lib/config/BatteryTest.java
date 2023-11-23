package org.team100.lib.config;

import static org.junit.jupiter.api.Assertions.assertEquals;

import org.junit.jupiter.api.Test;

class BatteryTest {
    @Test
    void testSimple() {
        BatteryFactory factory = new BatteryFactory(()->1);
        Battery battery = factory.get();
        assertEquals(Battery.Use.COMP, battery.use);
    }
    
}
