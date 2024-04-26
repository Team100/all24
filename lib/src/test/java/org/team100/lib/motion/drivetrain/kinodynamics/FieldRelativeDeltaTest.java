package org.team100.lib.motion.drivetrain.kinodynamics;

import static org.junit.jupiter.api.Assertions.assertEquals;

import org.junit.jupiter.api.Test;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

class FieldRelativeDeltaTest {
    @Test
    void testPolarity() {
        // the delta sign is correct
        Pose2d start = new Pose2d();
        Pose2d end = new Pose2d(1, 0, new Rotation2d());
        FieldRelativeDelta d = FieldRelativeDelta.delta(start, end);
        assertEquals(1, d.getTranslation().getX(), 0.01);
        assertEquals(0, d.getTranslation().getY(), 0.01);
        assertEquals(0, d.getRotation().getRadians(), 0.01);
    }

    @Test
    void testWithRotation() {
        // unlike Pose2d.minus(), the rotation is independent
        Pose2d start = new Pose2d();
        Pose2d end = new Pose2d(1, 0, new Rotation2d(1));
        FieldRelativeDelta d = FieldRelativeDelta.delta(start, end);
        assertEquals(1, d.getTranslation().getX(), 0.01);
        assertEquals(0, d.getTranslation().getY(), 0.01);
        assertEquals(1, d.getRotation().getRadians(), 0.01);
    }
}
