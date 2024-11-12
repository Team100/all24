package org.team100.lib.commands.drivetrain.for_testing;

import static org.junit.jupiter.api.Assertions.assertEquals;

import org.junit.jupiter.api.Test;
import org.team100.lib.motion.drivetrain.SwerveControl;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

class DriveInACircleTest {
    private static final double kDelta = 0.001;

    @Test
    void testCenters() {
        double radiusM = 1.0;

        // start at the origin, facing +x
        Pose2d initial = new Pose2d();
        Translation2d m_center = DriveInACircle.getCenter(initial, radiusM);
        assertEquals(0, m_center.getX(), kDelta);
        assertEquals(1, m_center.getY(), kDelta);

        // start at the origin, facing +y
        initial = new Pose2d(0, 0, new Rotation2d(0, 1));
        m_center = DriveInACircle.getCenter(initial, radiusM);
        // center should be to the left of that
        assertEquals(-1, m_center.getX(), kDelta);
        assertEquals(0, m_center.getY(), kDelta);

        // start at the origin, facing diagonal
        initial = new Pose2d(0, 0, new Rotation2d(Math.PI / 4));
        m_center = DriveInACircle.getCenter(initial, radiusM);
        assertEquals(-0.707, m_center.getX(), kDelta);
        assertEquals(0.707, m_center.getY(), kDelta);
    }

    @Test
    void testInitial() {
        double radiusM = 1.0;

        Pose2d initial = new Pose2d(); // start at the origin

        Translation2d m_center = DriveInACircle.getCenter(initial, radiusM);
        assertEquals(0, m_center.getX(), kDelta);
        assertEquals(1, m_center.getY(), kDelta);

        // just starting, zero velocity, max acceleration
        double m_angle = 0;
        double m_speed = 0;
        double accel = 1;
        double initialRotation = 0;
        double turnRatio = 0;

        SwerveControl reference = DriveInACircle.getReference(
                m_center,
                radiusM,
                m_angle,
                m_speed,
                accel,
                initialRotation,
                turnRatio);

        // reference location is current pose
        assertEquals(0, reference.x().x(), kDelta);
        assertEquals(0, reference.y().x(), kDelta);
        assertEquals(0, reference.theta().x(), kDelta);

        // reference velocity is zero
        assertEquals(0, reference.x().v(), kDelta);
        assertEquals(0, reference.y().v(), kDelta);
        assertEquals(0, reference.theta().v(), kDelta);

        // reference acceleration is +x
        assertEquals(1, reference.x().a(), kDelta);
        assertEquals(0, reference.y().a(), kDelta);
        assertEquals(0, reference.theta().a(), kDelta);
    }

    @Test
    void testInitialBigger() {
        double radiusM = 2.0;
        Pose2d initial = new Pose2d(); // start at the origin

        Translation2d m_center = DriveInACircle.getCenter(initial, radiusM);
        assertEquals(0, m_center.getX(), kDelta);
        assertEquals(2, m_center.getY(), kDelta);

        // just starting, zero velocity, max acceleration
        double m_angle = 0;
        double m_speed = 0;
        double accel = 1;
        double initialRotation = 0;
        double turnRatio = 0;

        SwerveControl reference = DriveInACircle.getReference(
                m_center,
                radiusM,
                m_angle,
                m_speed,
                accel,
                initialRotation,
                turnRatio);

        // reference location is current pose
        assertEquals(0, reference.x().x(), kDelta);
        assertEquals(0, reference.y().x(), kDelta);
        assertEquals(0, reference.theta().x(), kDelta);

        // reference velocity is zero
        assertEquals(0, reference.x().v(), kDelta);
        assertEquals(0, reference.y().v(), kDelta);
        assertEquals(0, reference.theta().v(), kDelta);

        // reference acceleration is +x
        assertEquals(1, reference.x().a(), kDelta);
        assertEquals(0, reference.y().a(), kDelta);
        assertEquals(0, reference.theta().a(), kDelta);
    }

    @Test
    void testInitialWithoutAcceleration() {
        double radiusM = 1.0;
        Pose2d initial = new Pose2d(); // start at the origin

        Translation2d m_center = DriveInACircle.getCenter(initial, radiusM);
        assertEquals(0, m_center.getX(), kDelta);
        assertEquals(1, m_center.getY(), kDelta);

        // this is like the beginning except we're already moving
        double m_angle = 0;
        double m_speed = 1;
        double accel = 0;
        double initialRotation = 0;
        double turnRatio = 0;

        SwerveControl reference = DriveInACircle.getReference(
                m_center,
                radiusM,
                m_angle,
                m_speed,
                accel,
                initialRotation,
                turnRatio);

        // reference location is current pose
        assertEquals(0, reference.x().x(), kDelta);
        assertEquals(0, reference.y().x(), kDelta);
        assertEquals(0, reference.theta().x(), kDelta);

        // reference velocity is +x
        assertEquals(1, reference.x().v(), kDelta);
        assertEquals(0, reference.y().v(), kDelta);
        assertEquals(0, reference.theta().v(), kDelta);

        // reference acceleration is +y
        assertEquals(0, reference.x().a(), kDelta);
        assertEquals(1, reference.y().a(), kDelta);
        assertEquals(0, reference.theta().a(), kDelta);
    }

    @Test
    void test90() {
        double radiusM = 1.0;
        Pose2d initial = new Pose2d(); // start at the origin

        Translation2d m_center = DriveInACircle.getCenter(initial, radiusM);
        assertEquals(0, m_center.getX(), kDelta);
        assertEquals(1, m_center.getY(), kDelta);

        // moving, should be heading in +y
        double m_angle = Math.PI / 2;
        double m_speed = 1;
        double accel = 0;
        double initialRotation = 0;
        double turnRatio = 0;

        SwerveControl reference = DriveInACircle.getReference(
                m_center,
                radiusM,
                m_angle,
                m_speed,
                accel,
                initialRotation,
                turnRatio);

        // reference location is at 90
        assertEquals(1, reference.x().x(), kDelta);
        assertEquals(1, reference.y().x(), kDelta);
        assertEquals(0, reference.theta().x(), kDelta);

        // reference velocity is +y
        assertEquals(0, reference.x().v(), kDelta);
        assertEquals(1, reference.y().v(), kDelta);
        assertEquals(0, reference.theta().v(), kDelta);

        // reference acceleration is -x
        assertEquals(-1, reference.x().a(), kDelta);
        assertEquals(0, reference.y().a(), kDelta);
        assertEquals(0, reference.theta().a(), kDelta);
    }

    @Test
    void test90Bigger() {
        double radiusM = 2.0;
        Pose2d initial = new Pose2d(); // start at the origin

        Translation2d m_center = DriveInACircle.getCenter(initial, radiusM);
        assertEquals(0, m_center.getX(), kDelta);
        assertEquals(2, m_center.getY(), kDelta);

        // moving, should be heading in +y
        double m_angle = Math.PI / 2;
        double m_speed = 1;
        double accel = 0;
        double initialRotation = 0;
        double turnRatio = 0;

        SwerveControl reference = DriveInACircle.getReference(
                m_center,
                radiusM,
                m_angle,
                m_speed,
                accel,
                initialRotation,
                turnRatio);

        // reference location is at 90
        assertEquals(2, reference.x().x(), kDelta);
        assertEquals(2, reference.y().x(), kDelta);
        assertEquals(0, reference.theta().x(), kDelta);

        // reference velocity is +y
        assertEquals(0, reference.x().v(), kDelta);
        assertEquals(2, reference.y().v(), kDelta);
        assertEquals(0, reference.theta().v(), kDelta);

        // reference acceleration is -x, less because it's a bigger circle.
        assertEquals(-2, reference.x().a(), kDelta);
        assertEquals(0, reference.y().a(), kDelta);
        assertEquals(0, reference.theta().a(), kDelta);
    }

    @Test
    void test90Faster() {
        double radiusM = 1.0;
        Pose2d initial = new Pose2d(); // start at the origin

        Translation2d m_center = DriveInACircle.getCenter(initial, radiusM);
        assertEquals(0, m_center.getX(), kDelta);
        assertEquals(1, m_center.getY(), kDelta);

        // moving, should be heading in +y
        double m_angle = Math.PI / 2;
        double m_speed = 2;
        double accel = 0;
        double initialRotation = 0;
        double turnRatio = 0;

        SwerveControl reference = DriveInACircle.getReference(
                m_center,
                radiusM,
                m_angle,
                m_speed,
                accel,
                initialRotation,
                turnRatio);

        // reference location is at 90
        assertEquals(1, reference.x().x(), kDelta);
        assertEquals(1, reference.y().x(), kDelta);
        assertEquals(0, reference.theta().x(), kDelta);

        // reference velocity is +y
        assertEquals(0, reference.x().v(), kDelta);
        assertEquals(2, reference.y().v(), kDelta);
        assertEquals(0, reference.theta().v(), kDelta);

        // reference acceleration is -x
        assertEquals(-4, reference.x().a(), kDelta);
        assertEquals(0, reference.y().a(), kDelta);
        assertEquals(0, reference.theta().a(), kDelta);
    }

    @Test
    void test180() {
        double radiusM = 1.0;
        Pose2d initial = new Pose2d(); // start at the origin

        Translation2d m_center = DriveInACircle.getCenter(initial, radiusM);
        assertEquals(0, m_center.getX(), kDelta);
        assertEquals(1, m_center.getY(), kDelta);

        // moving, should be heading in -x
        double m_angle = Math.PI;
        double m_speed = 1;
        double accel = 0;
        double initialRotation = 0;
        double turnRatio = 0;

        SwerveControl reference = DriveInACircle.getReference(
                m_center,
                radiusM,
                m_angle,
                m_speed,
                accel,
                initialRotation,
                turnRatio);

        // reference location is at 180
        assertEquals(0, reference.x().x(), kDelta);
        assertEquals(2, reference.y().x(), kDelta);
        assertEquals(0, reference.theta().x(), kDelta);

        // reference velocity is -x
        assertEquals(-1, reference.x().v(), kDelta);
        assertEquals(0, reference.y().v(), kDelta);
        assertEquals(0, reference.theta().v(), kDelta);

        // reference acceleration is -y
        assertEquals(0, reference.x().a(), kDelta);
        assertEquals(-1, reference.y().a(), kDelta);
        assertEquals(0, reference.theta().a(), kDelta);
    }

}
