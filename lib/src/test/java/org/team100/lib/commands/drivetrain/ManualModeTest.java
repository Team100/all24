package org.team100.lib.commands.drivetrain;

import static org.junit.jupiter.api.Assertions.assertEquals;

import org.junit.jupiter.api.Test;

class ManualModeTest {
    @Test
    void testSimple() {
        ManualMode mode = new ManualMode();
        // verify default
        assertEquals(ManualMode.Mode.FIELD_RELATIVE_TWIST, mode.getSelected());
    }
    
}
