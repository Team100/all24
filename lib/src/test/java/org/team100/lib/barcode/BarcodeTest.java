package org.team100.lib.barcode;

import static org.junit.jupiter.api.Assertions.assertEquals;

import org.junit.jupiter.api.Test;

public class BarcodeTest {
    @Test
    void testSimple() {
        Barcode barcode = new Barcode();
        barcode.robotInit();
        assertEquals(0, barcode.getAsInt());
        barcode.teleopPeriodic();
        barcode.testPeriodic();
        barcode.close();
    }
    
}
