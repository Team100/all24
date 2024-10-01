package org.team100.frc2024.motion.climber;

import static org.junit.jupiter.api.Assertions.assertEquals;

import org.junit.jupiter.api.Test;
import org.team100.frc2024.Timeless2024;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.logging.TestLoggerFactory;
import org.team100.lib.logging.primitive.TestPrimitiveLogger;

class ClimberSubsystemTest implements Timeless2024 {
    private static final double kDelta = 0.001;
    private static final LoggerFactory logger = new TestLoggerFactory(new TestPrimitiveLogger());

    @Test
    void testSimple() {
        ClimberSubsystem c = new ClimberSubsystem(logger, 0, 0);
        assertEquals(0, c.getLeft().getPositionM().getAsDouble(), kDelta);
        // run it for 0.25 sec at full output
        for (int i = 0; i < 12; ++i) {
            c.getLeft().setDutyCycle(1.0);
            stepTime(0.02);
        }
        // it's moved a lot
        assertEquals(0.085, c.getLeft().getPositionM().getAsDouble(), kDelta);
    }
}
