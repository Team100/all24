package frc.robot;

import static org.junit.jupiter.api.Assertions.assertEquals;

import org.dyn4j.geometry.Vector2;
import org.junit.jupiter.api.Test;
import org.team100.sim.Geometry;

class GeometryTest {
    private static final double kDelta = 0.001;

    @Test
    void testMotionless() {
        Vector2 position = new Vector2(0, 0);
        Vector2 velocity = new Vector2(0, 0);
        Vector2 targetPosition = new Vector2(1, 1);
        Vector2 closestApproach = Geometry.closestApproach(
                position,
                velocity,
                targetPosition);
        assertEquals(0, closestApproach.x, kDelta);
        assertEquals(0, closestApproach.y, kDelta);
    }

    @Test
    void testAhead() {
        Vector2 position = new Vector2(0, 0);
        Vector2 velocity = new Vector2(1, 0);
        Vector2 targetPosition = new Vector2(2, 0);
        Vector2 closestApproach = Geometry.closestApproach(
                position,
                velocity,
                targetPosition);
        assertEquals(2, closestApproach.x, kDelta);
        assertEquals(0, closestApproach.y, kDelta);
    }

    @Test
    void testRight() {
        Vector2 position = new Vector2(0, 0);
        Vector2 velocity = new Vector2(1, 0);
        Vector2 targetPosition = new Vector2(1, 2);
        Vector2 closestApproach = Geometry.closestApproach(
                position,
                velocity,
                targetPosition);
        assertEquals(1, closestApproach.x, kDelta);
        assertEquals(0, closestApproach.y, kDelta);
    }

    @Test
    void testBehind() {
        Vector2 position = new Vector2(0, 0);
        Vector2 velocity = new Vector2(1, 0);
        Vector2 targetPosition = new Vector2(-1, 2);
        Vector2 closestApproach = Geometry.closestApproach(
                position,
                velocity,
                targetPosition);
        assertEquals(-1, closestApproach.x, kDelta);
        assertEquals(0, closestApproach.y, kDelta);
    }

    @Test
    void testDiagonal() {
        Vector2 position = new Vector2(0, 0);
        Vector2 velocity = new Vector2(1, 1);
        Vector2 targetPosition = new Vector2(0, 2);
        Vector2 closestApproach = Geometry.closestApproach(
                position,
                velocity,
                targetPosition);
        assertEquals(1, closestApproach.x, kDelta);
        assertEquals(1, closestApproach.y, kDelta);
    }

    @Test
    void testAvoidAhead() {
        Vector2 position = new Vector2(0, 0);
        Vector2 velocity = new Vector2(1, 0);
        Vector2 targetPosition = new Vector2(2, 0);
        Vector2 steer = Geometry.steerToAvoid(
                position,
                velocity,
                targetPosition,
                1);
        // try to stop
        assertEquals(-1, steer.x, kDelta);
        assertEquals(0, steer.y, kDelta);
    }

    @Test
    void testAvoidFar() {
        Vector2 position = new Vector2(0, 0);
        Vector2 velocity = new Vector2(1, 0);
        Vector2 targetPosition = new Vector2(0, 2);
        Vector2 steer = Geometry.steerToAvoid(
                position,
                velocity,
                targetPosition,
                1);
        // no steering needed
        assertEquals(0, steer.x, kDelta);
        assertEquals(0, steer.y, kDelta);
    }

    @Test
    void testAvoidRight() {
        Vector2 position = new Vector2(0, 0);
        Vector2 velocity = new Vector2(1, 0);
        Vector2 targetPosition = new Vector2(0, -0.5);
        Vector2 steer = Geometry.steerToAvoid(
                position,
                velocity,
                targetPosition,
                1);
        // steer left to avoid target on the right
        assertEquals(0, steer.x, kDelta);
        assertEquals(0.5, steer.y, kDelta);
    }

    @Test
    void testAvoidDiagonal() {
        Vector2 position = new Vector2(0, 0);
        Vector2 velocity = new Vector2(1, 1);
        Vector2 targetPosition = new Vector2(1, 0);
        Vector2 steer = Geometry.steerToAvoid(
                position,
                velocity,
                targetPosition,
                1);
        // steer left to avoid target on the right
        assertEquals(-0.207, steer.x, kDelta);
        assertEquals(0.207, steer.y, kDelta);
    }
}
