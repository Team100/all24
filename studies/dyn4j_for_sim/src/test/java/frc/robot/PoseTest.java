package frc.robot;

import static org.junit.jupiter.api.Assertions.assertEquals;

import org.junit.jupiter.api.Test;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Twist2d;

class PoseTest {

    @Test
    void testMinus() {
        // transform2d is robot-relative,
        // so every time i use pose.minus(pose) i'm doing it wrong.
        Pose2d goal = new Pose2d(1, 1, new Rotation2d(1));
        Pose2d measurement = new Pose2d(2, 2, new Rotation2d(2));
        Transform2d transform = goal.minus(measurement);
        assertEquals(-0.49, transform.getX(), 0.01);
        assertEquals(1.32, transform.getY(), 0.01);
        assertEquals(-1, transform.getRotation().getRadians(), 0.1);
    }

    @Test
    void testRelativeTo() {
        // transform2d is robot-relative,
        // so every time i use pose.minus(pose) i'm doing it wrong.
        Pose2d goal = new Pose2d(1, 1, new Rotation2d(1));
        Pose2d measurement = new Pose2d(2, 2, new Rotation2d(2));
        Pose2d pose = goal.relativeTo(measurement);
        assertEquals(-0.49, pose.getX(), 0.01);
        assertEquals(1.32, pose.getY(), 0.01);
        assertEquals(-1, pose.getRotation().getRadians(), 0.1);
        // rotate by starting rotation to get field-relative translation
        Pose2d rotated = pose.rotateBy(new Rotation2d(2));
        assertEquals(-1, rotated.getX(), 0.01);
        assertEquals(-1, rotated.getY(), 0.01);
        assertEquals(1, rotated.getRotation().getRadians(), 0.01);
    }

    @Test
    void testLog() {
        // twist is robot-relative
        Pose2d goal = new Pose2d(1, 1, new Rotation2d(1));
        Pose2d measurement = new Pose2d(2, 2, new Rotation2d(2));
        Twist2d twist = measurement.log(goal);
        assertEquals(-1.11, twist.dx, 0.01);
        assertEquals(0.97, twist.dy, 0.01);
        assertEquals(-1, twist.dtheta, 0.1);
        // apply to pure rotation to get field relative
        Pose2d rotated = new Pose2d(0, 0, new Rotation2d(2)).exp(twist);
        assertEquals(-1, rotated.getX(), 0.01);
        assertEquals(-1, rotated.getY(), 0.01);
        assertEquals(1, rotated.getRotation().getRadians(), 0.01);
    }
}
