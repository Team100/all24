package org.team100.lib.motion.drivetrain;

import static org.junit.jupiter.api.Assertions.assertEquals;

import org.junit.jupiter.api.Test;

class VeeringCorrectionTest {
    private static final double kDelta = 0.001;

    @Test
    void testZero() {
        assertEquals(0, VeeringCorrection.correctionRad(0), kDelta);
    }

    @Test
    void testSpinning() {
        assertEquals(0.025, VeeringCorrection.correctionRad(1), kDelta);
    }
}