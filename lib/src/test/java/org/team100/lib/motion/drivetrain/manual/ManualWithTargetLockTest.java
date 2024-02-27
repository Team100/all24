package org.team100.lib.motion.drivetrain.manual;

import static org.junit.jupiter.api.Assertions.assertEquals;

import org.junit.jupiter.api.Test;
import org.team100.lib.motion.drivetrain.SwerveState;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;

class ManualWithTargetLockTest {
    private static final double kDelta = 0.001;

    @Test
    void testBearing() {
        assertEquals(0,
                ManualWithTargetLock.bearing(
                        new Translation2d(),
                        new Translation2d(1, 0)).getRadians(),
                kDelta);
        assertEquals(Math.PI / 2,
                ManualWithTargetLock.bearing(
                        new Translation2d(),
                        new Translation2d(0, 1)).getRadians(),
                kDelta);
        assertEquals(Math.PI / 4,
                ManualWithTargetLock.bearing(
                        new Translation2d(),
                        new Translation2d(1, 1)).getRadians(),
                kDelta);
        assertEquals(3 * Math.PI / 4,
                ManualWithTargetLock.bearing(
                        new Translation2d(),
                        new Translation2d(-1, 1)).getRadians(),
                kDelta);
        assertEquals(-Math.PI / 4,
                ManualWithTargetLock.bearing(
                        new Translation2d(),
                        new Translation2d(1, -1)).getRadians(),
                kDelta);
    }

    @Test
    void testTargetMotion() {
        // at the origin, moving 1m/s +x
        Pose2d p = new Pose2d();
        Twist2d t = new Twist2d(1, 0, 0);
        SwerveState state = new SwerveState(p, t);
        // target is 1m to the left
        Translation2d target = new Translation2d(0, 1);
        double motion = ManualWithTargetLock.targetMotion(state, target);
        assertEquals(1, motion, kDelta);
    }

    @Test
    void testTargetMotionReverse() {
        // at the origin, moving 1m/s +x
        Pose2d p = new Pose2d();
        Twist2d t = new Twist2d(1, 0, 0);
        SwerveState state = new SwerveState(p, t);
        // target is 1m to the right
        Translation2d target = new Translation2d(0, -1);
        double motion = ManualWithTargetLock.targetMotion(state, target);
        assertEquals(-1, motion, kDelta);
    }

    @Test
    void testTargetMotion2() {
        // at the origin, moving 1m/s +x
        Pose2d p = new Pose2d();
        Twist2d t = new Twist2d(1, 0, 0);
        SwerveState state = new SwerveState(p, t);
        // target is dead ahead
        Translation2d target = new Translation2d(2, 0);
        double motion = ManualWithTargetLock.targetMotion(state, target);
        assertEquals(0, motion, kDelta);
    }

    @Test
    void testTargetMotion3() {
        // at the origin, moving 1m/s +x
        Pose2d p = new Pose2d();
        Twist2d t = new Twist2d(1, 0, 0);
        SwerveState state = new SwerveState(p, t);
        // target is at 45
        Translation2d target = new Translation2d(1, 1);
        double motion = ManualWithTargetLock.targetMotion(state, target);
        assertEquals(0.5, motion, kDelta);
    }

    @Test
    void testTargetMotionY() {
        // at the origin, moving 1m/s +y
        Pose2d p = new Pose2d();
        Twist2d t = new Twist2d(0, 1, 0);
        SwerveState state = new SwerveState(p, t);
        // target is dead ahead
        Translation2d target = new Translation2d(1, 0);
        double motion = ManualWithTargetLock.targetMotion(state, target);
        assertEquals(-1, motion, kDelta);
    }
}
