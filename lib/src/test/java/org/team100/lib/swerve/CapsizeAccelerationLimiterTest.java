package org.team100.lib.swerve;

import static org.junit.jupiter.api.Assertions.assertEquals;

import org.junit.jupiter.api.Test;
import org.team100.lib.motion.drivetrain.kinodynamics.SwerveKinodynamics;
import org.team100.lib.motion.drivetrain.kinodynamics.SwerveKinodynamicsFactory;
import org.team100.lib.logging.TestLogger;
import org.team100.lib.logging.SupplierLogger2;

class CapsizeAccelerationLimiterTest {
    private static final double kDelta = 0.001;
    private static final SupplierLogger2 logger = new TestLogger().getSupplierLogger();

    /** zero delta v => no constraint */
    @Test
    void testUnconstrained() {
        SwerveKinodynamics l = SwerveKinodynamicsFactory.forTest();
        CapsizeAccelerationLimiter c = new CapsizeAccelerationLimiter(logger, l);
        double s = c.enforceCentripetalLimit(0, 0);
        assertEquals(1, s, kDelta);
    }

    /**
     * total delta v is 1.414 m/s.
     */
    @Test
    void testConstrained() {
        SwerveKinodynamics l = SwerveKinodynamicsFactory.forTest();
        CapsizeAccelerationLimiter c = new CapsizeAccelerationLimiter(logger, l);
        double s = c.enforceCentripetalLimit(-1, 1);
        assertEquals(0.115, s, kDelta);
    }
}
