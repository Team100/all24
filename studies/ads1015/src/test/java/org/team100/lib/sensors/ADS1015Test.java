package org.team100.lib.sensors;

import static org.junit.jupiter.api.Assertions.assertEquals;

import org.junit.jupiter.api.Test;

public class ADS1015Test {
    @Test
    void testADS1015() {
        ADS1015 ads1015 = new ADS1015();
        assertEquals(0, ads1015.readVolts(0), 0.001);
    }
    
}
