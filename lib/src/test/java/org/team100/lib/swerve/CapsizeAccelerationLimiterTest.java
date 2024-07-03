package org.team100.lib.swerve;

import static org.junit.jupiter.api.Assertions.assertEquals;

import org.junit.jupiter.api.Test;
import org.team100.lib.motion.drivetrain.kinodynamics.SwerveKinodynamics;
import org.team100.lib.motion.drivetrain.kinodynamics.SwerveKinodynamicsFactory;
import org.team100.lib.telemetry.Telemetry;
import org.team100.lib.telemetry.Telemetry.Logger;

class CapsizeAccelerationLimiterTest {
    private static final double kDelta = 0.001;

    /** zero delta v => no constraint */
    @Test
    void testUnconstrained() {
        SwerveKinodynamics l = SwerveKinodynamicsFactory.forTest();
         Logger logger = Telemetry.get().rootLogger("foo");
        CapsizeAccelerationLimiter c = new CapsizeAccelerationLimiter("foo", logger, l);
        double s = c.enforceCentripetalLimit(0, 0, 0.02);
        assertEquals(1, s, kDelta);
    }

    /**
     * total delta v is 1.414 m/s.
     */
    @Test
    void testConstrained() {
        SwerveKinodynamics l = SwerveKinodynamicsFactory.forTest();
        Logger logger = Telemetry.get().rootLogger("foo");
        CapsizeAccelerationLimiter c = new CapsizeAccelerationLimiter("foo", logger, l);
        double s = c.enforceCentripetalLimit(-1, 1, 0.02);
        assertEquals(0.115, s, kDelta);
    }
}
