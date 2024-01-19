package org.team100.lib.motion.arm;

import static org.junit.jupiter.api.Assertions.assertEquals;

import org.junit.jupiter.api.Test;
import org.team100.lib.testing.TimelessTest;

class ArmSubsystemTest extends TimelessTest {
    private static final double kDelta = 0.001;

    // test simple direct motion
    @Test
    void testSimple() {
        ArmSubsystem armSubSystem = ArmFactory.get();

        assertEquals(0, armSubSystem.getPosition().th1, kDelta);
        assertEquals(0, armSubSystem.getPosition().th2, kDelta);

        // short time to stay inside the physical limit
        for (int i = 0; i < 10; ++i) {
            stepTime(0.02);
            armSubSystem.set(1, 1);
            armSubSystem.periodic();
        }

        assertEquals(0.5, armSubSystem.getPosition().th1, 0.1);
        assertEquals(0.5, armSubSystem.getPosition().th2, 0.1);

        armSubSystem.close();
    }
}
