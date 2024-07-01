package org.team100.frc2024.motion.climber;

import static org.junit.jupiter.api.Assertions.assertEquals;

import org.junit.jupiter.api.Test;
import org.team100.frc2024.Timeless2024;

class ClimberSubsystemTest implements Timeless2024 {
    private static final double kDelta = 0.001;

    @Test
    void testSimple() {
        ClimberSubsystem c = new ClimberSubsystem(0, 0);
        assertEquals(0, c.getLeftPosition().getAsDouble(), kDelta);
        // run it for 0.25 sec at full output
        for (int i = 0; i < 12; ++i) {
            c.setLeft(1);
            stepTime(0.02);
        }
        // it's moved a lot
        assertEquals(0.24, c.getLeftPosition().getAsDouble(), kDelta);
    }
}
