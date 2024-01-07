package org.team100.lib.motion.drivetrain;

import static org.junit.jupiter.api.Assertions.assertEquals;

import org.junit.jupiter.api.Test;
import org.team100.lib.geometry.GeometryUtil;

class VeeringCorrectionTest {
    private static final double kDelta = 0.001;

    @Test
    void testZero() {
        assertEquals(0, VeeringCorrection.correct(0, GeometryUtil.kRotationZero).getRadians(), kDelta);
    }

    @Test
    void testSpinning() {
        assertEquals(0.05,
        VeeringCorrection.correct(1, GeometryUtil.kRotationZero).getRadians(), kDelta);
    }
}