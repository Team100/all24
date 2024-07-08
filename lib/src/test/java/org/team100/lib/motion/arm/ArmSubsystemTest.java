package org.team100.lib.motion.arm;

import static org.junit.jupiter.api.Assertions.assertEquals;

import org.junit.jupiter.api.Test;
import org.team100.lib.telemetry.Logger;
import org.team100.lib.telemetry.TestLogger;
import org.team100.lib.testing.Timeless;

class ArmSubsystemTest implements Timeless {
    private static final double kDelta = 0.001;
    private static final Logger logger = new TestLogger();

    // test simple direct motion
    @Test
    void testSimple() {
        ArmSubsystem armSubSystem = ArmFactory.get(logger);

        assertEquals(0, armSubSystem.getPosition().get().th1, kDelta);
        assertEquals(0, armSubSystem.getPosition().get().th2, kDelta);

        for (int i = 0; i < 10; ++i) {
            stepTime(0.02);
            armSubSystem.set(1, 1);
        }

        // these values seem timing dependent?
        assertEquals(0.2, armSubSystem.getPosition().get().th1, 0.2);
        assertEquals(0.3, armSubSystem.getPosition().get().th2, 0.2);

        armSubSystem.close();
    }
}
