package org.team100.lib.motion.drivetrain.manual;

import static org.junit.jupiter.api.Assertions.assertEquals;

import org.junit.jupiter.api.Test;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

class ManualWithTargetLockTest {
    private static final double kDelta = 0.001;

    @Test
    void testAngle() {
        assertEquals(0,
                ManualWithTargetLock.fieldRelativeAngleToTarget(
                        new Translation2d(),
                        new Translation2d(1, 0)).getRadians(),
                kDelta);
        assertEquals(Math.PI / 2,
                ManualWithTargetLock.fieldRelativeAngleToTarget(
                        new Translation2d(),
                        new Translation2d(0, 1)).getRadians(),
                kDelta);
        assertEquals(Math.PI / 4,
                ManualWithTargetLock.fieldRelativeAngleToTarget(
                        new Translation2d(),
                        new Translation2d(1, 1)).getRadians(),
                kDelta);
        assertEquals(3 * Math.PI / 4,
                ManualWithTargetLock.fieldRelativeAngleToTarget(
                        new Translation2d(),
                        new Translation2d(-1, 1)).getRadians(),
                kDelta);
        assertEquals(-Math.PI / 4,
                ManualWithTargetLock.fieldRelativeAngleToTarget(
                        new Translation2d(),
                        new Translation2d(1, -1)).getRadians(),
                kDelta);
    }

}
