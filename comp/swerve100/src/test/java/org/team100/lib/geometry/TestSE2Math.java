package org.team100.lib.geometry;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.junit.jupiter.api.Test;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;

class TestSE2Math {
    private static final double kTestEpsilon = 1e-12;

    @Test
    void testRotation2d() {
        // Test constructors
        Rotation2d rot1 = new Rotation2d();
        assertEquals(1, rot1.getCos(), kTestEpsilon);
        assertEquals(0, rot1.getSin(), kTestEpsilon);
        assertEquals(0, rot1.getTan(), kTestEpsilon);
        assertEquals(0, rot1.getDegrees(), kTestEpsilon);
        assertEquals(0, rot1.getRadians(), kTestEpsilon);

        rot1 = new Rotation2d(1, 1);
        assertEquals(Math.sqrt(2) / 2, rot1.getCos(), kTestEpsilon);
        assertEquals(Math.sqrt(2) / 2, rot1.getSin(), kTestEpsilon);
        assertEquals(1, rot1.getTan(), kTestEpsilon);
        assertEquals(45, rot1.getDegrees(), kTestEpsilon);
        assertEquals(Math.PI / 4, rot1.getRadians(), kTestEpsilon);

        rot1 = Rotation2d.fromRadians(Math.PI / 2);
        assertEquals(0, rot1.getCos(), kTestEpsilon);
        assertEquals(1, rot1.getSin(), kTestEpsilon);
        assertTrue(1 / kTestEpsilon < rot1.getTan());
        assertEquals(90, rot1.getDegrees(), kTestEpsilon);
        assertEquals(Math.PI / 2, rot1.getRadians(), kTestEpsilon);

        rot1 = Rotation2d.fromDegrees(270);
        assertEquals(0, rot1.getCos(), kTestEpsilon);
        assertEquals(-1, rot1.getSin(), kTestEpsilon);
        // System.out.println(rot1.getTan());
        // this test is silly
        // assertTrue(-1 / kTestEpsilon > rot1.getTan(), String.format("%f",
        // rot1.getTan()));
        // this tests the angle-wrapping thing that wpi doesn't do
        // assertEquals(-90, rot1.getDegrees(), kTestEpsilon);
        assertEquals(270, rot1.getDegrees(), kTestEpsilon);
        assertEquals(3 * Math.PI / 2, rot1.getRadians(), kTestEpsilon);

        // Test inversion
        rot1 = Rotation2d.fromDegrees(270);
        Rotation2d rot2 = rot1.unaryMinus();
        assertEquals(0, rot2.getCos(), kTestEpsilon);
        assertEquals(1, rot2.getSin(), kTestEpsilon);
        // this test is silly
        // assertTrue(1 / kTestEpsilon < rot2.getTan());
        // this tests the angle-wrapping thing that wpi doesn't do
        //assertEquals(90, rot2.getDegrees(), kTestEpsilon);
        assertEquals(-270, rot2.getDegrees(), kTestEpsilon);
        assertEquals(-3*Math.PI / 2, rot2.getRadians(), kTestEpsilon);

        rot1 = Rotation2d.fromDegrees(1);
        rot2 = rot1.unaryMinus();
        assertEquals(rot1.getCos(), rot2.getCos(), kTestEpsilon);
        assertEquals(-rot1.getSin(), rot2.getSin(), kTestEpsilon);
        assertEquals(-1, rot2.getDegrees(), kTestEpsilon);

        // Test rotateBy
        rot1 = Rotation2d.fromDegrees(45);
        rot2 = Rotation2d.fromDegrees(45);
        Rotation2d rot3 = rot1.rotateBy(rot2);
        assertEquals(0, rot3.getCos(), kTestEpsilon);
        assertEquals(1, rot3.getSin(), kTestEpsilon);
        assertTrue(1 / kTestEpsilon < rot3.getTan());
        assertEquals(90, rot3.getDegrees(), kTestEpsilon);
        assertEquals(Math.PI / 2, rot3.getRadians(), kTestEpsilon);

        rot1 = Rotation2d.fromDegrees(45);
        rot2 = Rotation2d.fromDegrees(-45);
        rot3 = rot1.rotateBy(rot2);
        assertEquals(1, rot3.getCos(), kTestEpsilon);
        assertEquals(0, rot3.getSin(), kTestEpsilon);
        assertEquals(0, rot3.getTan(), kTestEpsilon);
        assertEquals(0, rot3.getDegrees(), kTestEpsilon);
        assertEquals(0, rot3.getRadians(), kTestEpsilon);

        // A rotation times its inverse should be the identity
        Rotation2d identity = new Rotation2d();
        rot1 = Rotation2d.fromDegrees(21.45);
        rot2 = rot1.rotateBy(rot1.unaryMinus());
        assertEquals(identity.getCos(), rot2.getCos(), kTestEpsilon);
        assertEquals(identity.getSin(), rot2.getSin(), kTestEpsilon);
        assertEquals(identity.getDegrees(), rot2.getDegrees(), kTestEpsilon);

        // Test interpolation
        rot1 = Rotation2d.fromDegrees(45);
        rot2 = Rotation2d.fromDegrees(135);
        rot3 = rot1.interpolate(rot2, .5);
        assertEquals(90, rot3.getDegrees(), kTestEpsilon);

        rot1 = Rotation2d.fromDegrees(45);
        rot2 = Rotation2d.fromDegrees(135);
        rot3 = rot1.interpolate(rot2, .75);
        assertEquals(112.5, rot3.getDegrees(), kTestEpsilon);

        rot1 = Rotation2d.fromDegrees(45);
        rot2 = Rotation2d.fromDegrees(-45);
        rot3 = rot1.interpolate(rot2, .5);
        assertEquals(0, rot3.getDegrees(), kTestEpsilon);

        rot1 = Rotation2d.fromDegrees(45);
        rot2 = Rotation2d.fromDegrees(45);
        rot3 = rot1.interpolate(rot2, .5);
        assertEquals(45, rot3.getDegrees(), kTestEpsilon);

        rot1 = Rotation2d.fromDegrees(45);
        rot2 = Rotation2d.fromDegrees(45);
        rot3 = rot1.interpolate(rot2, .5);
        assertEquals(45, rot3.getDegrees(), kTestEpsilon);

        // Test parallel.
        rot1 = Rotation2d.fromDegrees(45);
        rot2 = Rotation2d.fromDegrees(45);
        assertTrue(GeometryUtil.isParallel(rot1, rot2));

        rot1 = Rotation2d.fromDegrees(45);
        rot2 = Rotation2d.fromDegrees(-45);
        assertFalse(GeometryUtil.isParallel(rot1, rot2));

        rot1 = Rotation2d.fromDegrees(45);
        rot2 = Rotation2d.fromDegrees(-135);
        assertTrue(GeometryUtil.isParallel(rot1, rot2));
    }

    @Test
    void testTranslation2d() {
        // Test constructors
        Translation2d pos1 = new Translation2d();
        assertEquals(0, pos1.getX(), kTestEpsilon);
        assertEquals(0, pos1.getY(), kTestEpsilon);
        assertEquals(0, pos1.getNorm(), kTestEpsilon);

        pos1 = new Translation2d(3, 4);
        assertEquals(3, pos1.getX(), kTestEpsilon);
        assertEquals(4, pos1.getY(), kTestEpsilon);
        assertEquals(5, pos1.getNorm(), kTestEpsilon);

        // Test inversion
        pos1 = new Translation2d(3.152, 4.1666);
        Translation2d pos2 = pos1.unaryMinus();
        assertEquals(-pos1.getX(), pos2.getX(), kTestEpsilon);
        assertEquals(-pos1.getY(), pos2.getY(), kTestEpsilon);
        assertEquals(pos1.getNorm(), pos2.getNorm(), kTestEpsilon);

        // Test rotateBy
        pos1 = new Translation2d(2, 0);
        Rotation2d rot1 = Rotation2d.fromDegrees(90);
        pos2 = pos1.rotateBy(rot1);
        assertEquals(0, pos2.getX(), kTestEpsilon);
        assertEquals(2, pos2.getY(), kTestEpsilon);
        assertEquals(pos1.getNorm(), pos2.getNorm(), kTestEpsilon);

        pos1 = new Translation2d(2, 0);
        rot1 = Rotation2d.fromDegrees(-45);
        pos2 = pos1.rotateBy(rot1);
        assertEquals(Math.sqrt(2), pos2.getX(), kTestEpsilon);
        assertEquals(-Math.sqrt(2), pos2.getY(), kTestEpsilon);
        assertEquals(pos1.getNorm(), pos2.getNorm(), kTestEpsilon);

        // Test translateBy
        pos1 = new Translation2d(2, 0);
        pos2 = new Translation2d(-2, 1);
        Translation2d pos3 = pos1.plus(pos2);
        assertEquals(0, pos3.getX(), kTestEpsilon);
        assertEquals(1, pos3.getY(), kTestEpsilon);
        assertEquals(1, pos3.getNorm(), kTestEpsilon);

        // A translation times its inverse should be the identity
        Translation2d identity = new Translation2d();
        pos1 = new Translation2d(2.16612, -23.55);
        pos2 = pos1.plus(pos1.unaryMinus());
        assertEquals(identity.getX(), pos2.getX(), kTestEpsilon);
        assertEquals(identity.getY(), pos2.getY(), kTestEpsilon);
        assertEquals(identity.getNorm(), pos2.getNorm(), kTestEpsilon);

        // Test interpolation
        pos1 = new Translation2d(0, 1);
        pos2 = new Translation2d(10, -1);
        pos3 = pos1.interpolate(pos2, .5);
        assertEquals(5, pos3.getX(), kTestEpsilon);
        assertEquals(0, pos3.getY(), kTestEpsilon);

        pos1 = new Translation2d(0, 1);
        pos2 = new Translation2d(10, -1);
        pos3 = pos1.interpolate(pos2, .75);
        assertEquals(7.5, pos3.getX(), kTestEpsilon);
        assertEquals(-.5, pos3.getY(), kTestEpsilon);
    }

    @Test
    void testPose2d() {
        // Test constructors
        Pose2d pose1 = new Pose2d();
        assertEquals(0, pose1.getTranslation().getX(), kTestEpsilon);
        assertEquals(0, pose1.getTranslation().getY(), kTestEpsilon);
        assertEquals(0, pose1.getRotation().getDegrees(), kTestEpsilon);

        pose1 = new Pose2d(new Translation2d(3, 4), Rotation2d.fromDegrees(45));
        assertEquals(3, pose1.getTranslation().getX(), kTestEpsilon);
        assertEquals(4, pose1.getTranslation().getY(), kTestEpsilon);
        assertEquals(45, pose1.getRotation().getDegrees(), kTestEpsilon);

        // Test transformation
        pose1 = new Pose2d(new Translation2d(3, 4), Rotation2d.fromDegrees(90));
        Pose2d pose2 = new Pose2d(new Translation2d(1, 0), Rotation2d.fromDegrees(0));
        Pose2d pose3 = GeometryUtil.transformBy(pose1, pose2);
        assertEquals(3, pose3.getTranslation().getX(), kTestEpsilon);
        assertEquals(5, pose3.getTranslation().getY(), kTestEpsilon);
        assertEquals(90, pose3.getRotation().getDegrees(), kTestEpsilon);

        pose1 = new Pose2d(new Translation2d(3, 4), Rotation2d.fromDegrees(90));
        pose2 = new Pose2d(new Translation2d(1, 0), Rotation2d.fromDegrees(-90));
        pose3 = GeometryUtil.transformBy(pose1, pose2);
        assertEquals(3, pose3.getTranslation().getX(), kTestEpsilon);
        assertEquals(5, pose3.getTranslation().getY(), kTestEpsilon);
        assertEquals(0, pose3.getRotation().getDegrees(), kTestEpsilon);

        // A pose times its inverse should be the identity
        Pose2d identity = new Pose2d();
        pose1 = new Pose2d(new Translation2d(3.51512152, 4.23), Rotation2d.fromDegrees(91.6));
        pose2 = GeometryUtil.transformBy(pose1, GeometryUtil.inverse(pose1));
        assertEquals(identity.getTranslation().getX(), pose2.getTranslation().getX(), kTestEpsilon);
        assertEquals(identity.getTranslation().getY(), pose2.getTranslation().getY(), kTestEpsilon);
        assertEquals(identity.getRotation().getDegrees(), pose2.getRotation().getDegrees(), kTestEpsilon);

        // Test interpolation
        // Movement from pose1 to pose2 is along a circle with radius of 10 units
        // centered at (3, -6)
        pose1 = new Pose2d(new Translation2d(3, 4), Rotation2d.fromDegrees(90));
        pose2 = new Pose2d(new Translation2d(13, -6), Rotation2d.fromDegrees(0.0));
        pose3 = pose1.interpolate(pose2, .5);
        double expected_angle_rads = Math.PI / 4;
        assertEquals(3.0 + 10.0 * Math.cos(expected_angle_rads), pose3.getTranslation().getX(), kTestEpsilon);
        assertEquals(-6.0 + 10.0 * Math.sin(expected_angle_rads), pose3.getTranslation().getY(), kTestEpsilon);
        assertEquals(expected_angle_rads, pose3.getRotation().getRadians(), kTestEpsilon);

        pose1 = new Pose2d(new Translation2d(3, 4), Rotation2d.fromDegrees(90));
        pose2 = new Pose2d(new Translation2d(13, -6), Rotation2d.fromDegrees(0.0));
        pose3 = pose1.interpolate(pose2, .75);
        expected_angle_rads = Math.PI / 8;
        assertEquals(3.0 + 10.0 * Math.cos(expected_angle_rads), pose3.getTranslation().getX(), kTestEpsilon);
        assertEquals(-6.0 + 10.0 * Math.sin(expected_angle_rads), pose3.getTranslation().getY(), kTestEpsilon);
        assertEquals(expected_angle_rads, pose3.getRotation().getRadians(), kTestEpsilon);
    }

    @Test
    void testTwist() {
        // Exponentiation (integrate twist to obtain a Pose2d)
        Twist2d twist = new Twist2d(1.0, 0.0, 0.0);
        Pose2d pose = new Pose2d().exp(twist);
        assertEquals(1.0, pose.getTranslation().getX(), kTestEpsilon);
        assertEquals(0.0, pose.getTranslation().getY(), kTestEpsilon);
        assertEquals(0.0, pose.getRotation().getDegrees(), kTestEpsilon);

        // Scaled.
        twist = new Twist2d(1.0, 0.0, 0.0);
        pose = new Pose2d().exp(GeometryUtil.scale(twist, 2.5));
        assertEquals(2.5, pose.getTranslation().getX(), kTestEpsilon);
        assertEquals(0.0, pose.getTranslation().getY(), kTestEpsilon);
        assertEquals(0.0, pose.getRotation().getDegrees(), kTestEpsilon);

        // Logarithm (find the twist to apply to obtain a given Pose2d)
        pose = new Pose2d(new Translation2d(2.0, 2.0), Rotation2d.fromRadians(Math.PI / 2));
        twist = new Pose2d().log(pose);
        assertEquals(Math.PI, twist.dx, kTestEpsilon);
        assertEquals(0.0, twist.dy, kTestEpsilon);
        assertEquals(Math.PI / 2, twist.dtheta, kTestEpsilon);

        // Logarithm is the inverse of exponentiation.
        Pose2d new_pose = new Pose2d().exp(twist);
        assertEquals(new_pose.getTranslation().getX(), pose.getTranslation().getX(), kTestEpsilon);
        assertEquals(new_pose.getTranslation().getY(), pose.getTranslation().getY(), kTestEpsilon);
        assertEquals(new_pose.getRotation().getDegrees(), pose.getRotation().getDegrees(), kTestEpsilon);
    }
}
