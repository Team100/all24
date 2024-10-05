package org.team100.lib.planner;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertNull;

import org.junit.jupiter.api.Test;
import org.team100.lib.motion.drivetrain.kinodynamics.FieldRelativeVelocity;

import edu.wpi.first.math.geometry.Translation2d;

class HeuristicsTest {
    private static final double kDelta = 0.001;
    Heuristics m_heuristics = new Heuristics(false);

    @Test
    void testMotionless() {
        Translation2d position = new Translation2d(0, 0);
        FieldRelativeVelocity velocity = new FieldRelativeVelocity(0, 0, 0);
        Translation2d targetPosition = new Translation2d(1, 1);
        Translation2d closestApproach = m_heuristics.closestApproach(
                position,
                velocity,
                targetPosition);
        assertNull(closestApproach);
    }

    @Test
    void testAhead() {
        Translation2d position = new Translation2d(0, 0);
        FieldRelativeVelocity velocity = new FieldRelativeVelocity(1, 0, 0);
        Translation2d targetPosition = new Translation2d(2, 0);
        Translation2d closestApproach = m_heuristics.closestApproach(
                position,
                velocity,
                targetPosition);
        assertEquals(2, closestApproach.getX(), kDelta);
        assertEquals(0, closestApproach.getY(), kDelta);
    }

    @Test
    void testRight() {
        Translation2d position = new Translation2d(0, 0);
        FieldRelativeVelocity velocity = new FieldRelativeVelocity(1, 0, 0);
        Translation2d targetPosition = new Translation2d(1, 2);
        Translation2d closestApproach = m_heuristics.closestApproach(
                position,
                velocity,
                targetPosition);
        assertEquals(1, closestApproach.getX(), kDelta);
        assertEquals(0, closestApproach.getY(), kDelta);
    }

    @Test
    void testBehind() {
        Translation2d position = new Translation2d(0, 0);
        FieldRelativeVelocity velocity = new FieldRelativeVelocity(1, 0, 0);
        Translation2d targetPosition = new Translation2d(-1, 2);
        Translation2d closestApproach = m_heuristics.closestApproach(
                position,
                velocity,
                targetPosition);
        // target is behind, closest approach is where we are now
        assertNull(closestApproach);
    }

    @Test
    void testDiagonal() {
        Translation2d position = new Translation2d(0, 0);
        FieldRelativeVelocity velocity = new FieldRelativeVelocity(1, 1, 0);
        Translation2d targetPosition = new Translation2d(0, 2);
        Translation2d closestApproach = m_heuristics.closestApproach(
                position,
                velocity,
                targetPosition);
        assertEquals(1, closestApproach.getX(), kDelta);
        assertEquals(1, closestApproach.getY(), kDelta);
    }

    @Test
    void testAvoidAhead() {
        Translation2d position = new Translation2d(0, 0);
        FieldRelativeVelocity velocity = new FieldRelativeVelocity(1, 0, 0);
        Translation2d targetPosition = new Translation2d(2, 0);
        FieldRelativeVelocity steer = m_heuristics.steerToAvoid(
                position,
                velocity,
                targetPosition,
                1);
        // try to stop
        assertEquals(-1, steer.x(), kDelta);
        assertEquals(0, steer.y(), kDelta);
    }

    @Test
    void testAvoidFar() {
        Translation2d position = new Translation2d(0, 0);
        FieldRelativeVelocity velocity = new FieldRelativeVelocity(1, 0, 0);
        Translation2d targetPosition = new Translation2d(0, 2);
        FieldRelativeVelocity steer = m_heuristics.steerToAvoid(
                position,
                velocity,
                targetPosition,
                1);
        // no steering needed
        assertEquals(0, steer.x(), kDelta);
        assertEquals(0, steer.y(), kDelta);
    }

    @Test
    void testAvoidRightAhead() {
        Translation2d position = new Translation2d(0, 0);
        FieldRelativeVelocity velocity = new FieldRelativeVelocity(1, 0, 0);
        Translation2d targetPosition = new Translation2d(1, -0.5);
        FieldRelativeVelocity steer = m_heuristics.steerToAvoid(
                position,
                velocity,
                targetPosition,
                1);
        // steer left to avoid target on the right
        assertEquals(0, steer.x(), kDelta);
        assertEquals(0.5, steer.y(), kDelta);
    }

    @Test
    void testAvoidRightAheadMotionless() {
        Translation2d position = new Translation2d(0, 0);
        FieldRelativeVelocity velocity = new FieldRelativeVelocity(0, 0, 0);
        Translation2d targetPosition = new Translation2d(1, -0.5);
        FieldRelativeVelocity steer = m_heuristics.steerToAvoid(
                position,
                velocity,
                targetPosition,
                1);
        // steer left to avoid target on the right
        assertEquals(0, steer.x(), kDelta);
        assertEquals(0, steer.y(), kDelta);
    }

    @Test
    void testAvoidRightAbeamMotionless() {
        Translation2d position = new Translation2d(0, 0);
        FieldRelativeVelocity velocity = new FieldRelativeVelocity(0, 0, 0);
        Translation2d targetPosition = new Translation2d(0, -0.5);
        FieldRelativeVelocity steer = m_heuristics.steerToAvoid(
                position,
                velocity,
                targetPosition,
                1);
        // move directly away
        assertEquals(0, steer.x(), kDelta);
        assertEquals(0, steer.y(), kDelta);
    }

    @Test
    void testAvoidRightAbeam() {
        Translation2d position = new Translation2d(0, 0);
        FieldRelativeVelocity velocity = new FieldRelativeVelocity(1, 0, 0);
        Translation2d targetPosition = new Translation2d(0, -0.5);
        FieldRelativeVelocity steer = m_heuristics.steerToAvoid(
                position,
                velocity,
                targetPosition,
                1);
        // already moving away, no need for more
        assertEquals(0, steer.x(), kDelta);
        assertEquals(0, steer.y(), kDelta);
    }

    @Test
    void testAvoidRightBehind() {
        Translation2d position = new Translation2d(0, 0);
        FieldRelativeVelocity velocity = new FieldRelativeVelocity(1, 0, 0);
        Translation2d targetPosition = new Translation2d(-1, -0.5);
        FieldRelativeVelocity steer = m_heuristics.steerToAvoid(
                position,
                velocity,
                targetPosition,
                1);
        // we've passed it, no steering
        assertEquals(0, steer.x(), kDelta);
        assertEquals(0, steer.y(), kDelta);
    }

    @Test
    void testAvoidDiagonal() {
        Translation2d position = new Translation2d(0, 0);
        FieldRelativeVelocity velocity = new FieldRelativeVelocity(1, 1, 0);
        Translation2d targetPosition = new Translation2d(1, 0);
        FieldRelativeVelocity steer = m_heuristics.steerToAvoid(
                position,
                velocity,
                targetPosition,
                1);
        // steer left to avoid target on the right
        assertEquals(-0.414, steer.x(), kDelta);
        assertEquals(0.414, steer.y(), kDelta);
    }
}
