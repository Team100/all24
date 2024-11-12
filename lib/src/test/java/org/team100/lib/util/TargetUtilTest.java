package org.team100.lib.util;

import static org.junit.jupiter.api.Assertions.assertEquals;

import org.junit.jupiter.api.Test;
import org.team100.lib.geometry.GeometryUtil;
import org.team100.lib.geometry.TargetUtil;
import org.team100.lib.motion.drivetrain.SwerveModel;
import org.team100.lib.motion.drivetrain.kinodynamics.FieldRelativeVelocity;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

class TargetUtilTest {

    private static final double kDelta = 0.001;

    @Test
    void testBearing() {
        assertEquals(0,
                TargetUtil.bearing(
                        new Translation2d(),
                        new Translation2d(1, 0)).getRadians(),
                kDelta);
        assertEquals(Math.PI / 2,
                TargetUtil.bearing(
                        new Translation2d(),
                        new Translation2d(0, 1)).getRadians(),
                kDelta);
        assertEquals(Math.PI / 4,
                TargetUtil.bearing(
                        new Translation2d(),
                        new Translation2d(1, 1)).getRadians(),
                kDelta);
        assertEquals(3 * Math.PI / 4,
                TargetUtil.bearing(
                        new Translation2d(),
                        new Translation2d(-1, 1)).getRadians(),
                kDelta);
        assertEquals(-Math.PI / 4,
                TargetUtil.bearing(
                        new Translation2d(),
                        new Translation2d(1, -1)).getRadians(),
                kDelta);
    }

    @Test
    void testTargetMotion() {
        // at the origin moving 1 m/s +x
        SwerveModel state = new SwerveModel(new Pose2d(), new FieldRelativeVelocity(1, 0, 0));
        // target is 1m to the left
        Translation2d target = new Translation2d(0, 1);
        // so it appears to move clockwise
        assertEquals(1, TargetUtil.targetMotion(state, target), kDelta);
    }

    @Test
    void testTargetMotionFaster() {
        // at the origin moving 2 m/s +x
        SwerveModel state = new SwerveModel(new Pose2d(), new FieldRelativeVelocity(2, 0, 0));
        // target is 1m to the left
        Translation2d target = new Translation2d(0, 1);
        // so it appears to move clockwise
        assertEquals(2, TargetUtil.targetMotion(state, target), kDelta);
    }

    @Test
    void testTargetMotionElsewhere() {
        // somewhere else, moving 1 m/s +x
        SwerveModel state = new SwerveModel(new Pose2d(1, 1, new Rotation2d()), new FieldRelativeVelocity(1, 0, 0));
        // target is 1m to the left
        Translation2d target = new Translation2d(1, 2);
        // so it appears to move clockwise
        assertEquals(1, TargetUtil.targetMotion(state, target), kDelta);
    }

    @Test
    void testTargetMotionReverse() {
        // at the origin, moving 1m/s +x
        SwerveModel state = new SwerveModel(new Pose2d(), new FieldRelativeVelocity(1, 0, 0));
        // target is 1m to the right
        Translation2d target = new Translation2d(0, -1);
        // so it appears to move counterclockwise
        assertEquals(-1, TargetUtil.targetMotion(state, target), kDelta);
    }

    @Test
    void testTargetMotionAhead() {
        // at the origin, moving 1m/s +x
        SwerveModel state = new SwerveModel(new Pose2d(), new FieldRelativeVelocity(1, 0, 0));
        // target is dead ahead
        Translation2d target = new Translation2d(2, 0);
        // no apparent motion
        assertEquals(0, TargetUtil.targetMotion(state, target), kDelta);
    }

    @Test
    void testTargetMotionOblique() {
        // at the origin, moving 1m/s +x
        SwerveModel state = new SwerveModel(new Pose2d(), new FieldRelativeVelocity(1, 0, 0));
        // target is at 45
        Translation2d target = new Translation2d(1, 1);
        // apparent motion is slower
        assertEquals(0.5, TargetUtil.targetMotion(state, target), kDelta);
    }

    @Test
    void testTargetMotionY() {
        // at the origin, moving 1m/s +y
        SwerveModel state = new SwerveModel(new Pose2d(), new FieldRelativeVelocity(0, 1, 0));
        // target is dead ahead
        Translation2d target = new Translation2d(1, 0);
        // target moves the other way
        assertEquals(-1, TargetUtil.targetMotion(state, target), kDelta);
    }

    @Test
    void testTargetMotionYReversed() {
        // in front of the origin, facing back to it, moving 1m/s +y,
        SwerveModel state = new SwerveModel(
                new Pose2d(1, 0, GeometryUtil.kRotation180),
                new FieldRelativeVelocity(0, 1, 0));
        // target is dead ahead
        Translation2d target = new Translation2d(0, 0);
        // target appears to move counterclockwise
        assertEquals(1, TargetUtil.targetMotion(state, target), kDelta);
    }
}
