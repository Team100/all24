package org.team100.frc2023.localization;

import static org.junit.jupiter.api.Assertions.assertEquals;

import org.junit.jupiter.api.Test;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.numbers.N3;

/**
 * Remind myself how 3d rotations work.
 */
public class CameraTest {
    private static final double kDelta = 0.01;

    @Test
    public void testNoRotation() {
        double roll = 0.0; // around X (ahead)
        double pitch = 0.0; // around Y (left)
        double yaw = 0.0; // around Z (up)
        Rotation3d r = new Rotation3d(roll, pitch, yaw);
        assertEquals(0, r.getX(), kDelta);
        assertEquals(0, r.getY(), kDelta);
        assertEquals(0, r.getZ(), kDelta);
    }

    @Test
    public void testRoll() {
        double roll = 1.0; // clockwise
        double pitch = 0.0;
        double yaw = 0.0;
        Rotation3d r = new Rotation3d(roll, pitch, yaw);
        assertEquals(1.0, r.getX(), kDelta);
        assertEquals(0, r.getY(), kDelta);
        assertEquals(0, r.getZ(), kDelta);
    }

    @Test
    public void testPitch() {
        double roll = 0.0;
        double pitch = 1.0; // down
        double yaw = 0.0;
        Rotation3d r = new Rotation3d(roll, pitch, yaw);
        assertEquals(0, r.getX(), kDelta);
        assertEquals(1.0, r.getY(), kDelta);
        assertEquals(0, r.getZ(), kDelta);
    }

    @Test
    public void testYaw() {
        double roll = 0.0;
        double pitch = 0.0;
        double yaw = 1.0; // left
        Rotation3d r = new Rotation3d(roll, pitch, yaw);
        assertEquals(0, r.getX(), kDelta);
        assertEquals(0, r.getY(), kDelta);
        assertEquals(1.0, r.getZ(), kDelta);
    }

    @Test
    public void testIMatrix() {
        Matrix<N3, N3> matrix = Matrix.eye(Nat.N3());
        Rotation3d r = new Rotation3d(matrix);
        assertEquals(0, r.getX(), kDelta);
        assertEquals(0, r.getY(), kDelta);
        assertEquals(0, r.getZ(), kDelta);
    }

    @Test
    public void testRollMatrix() {
        // in FRC X is the roll axis, positive roll is clockwise looking in the X
        // direction
        Matrix<N3, N3> matrix = new Matrix<>(Nat.N3(), Nat.N3());
        double rot = Math.sqrt(2) / 2;
        /*
         * [[1.0, 0.0, 0.0],
         * [0.0, 0.7, -0.7],
         * [0.0, 0.7, 0.7]]
         */
        matrix.set(0, 0, 1);
        matrix.set(0, 1, 0);
        matrix.set(0, 2, 0);

        matrix.set(1, 0, 0);
        matrix.set(1, 1, rot);
        matrix.set(1, 2, -rot);

        matrix.set(2, 0, 0);
        matrix.set(2, 1, rot);
        matrix.set(2, 2, rot);

        Rotation3d r = new Rotation3d(matrix);
        assertEquals(Math.PI / 4, r.getX(), kDelta);
        assertEquals(0, r.getY(), kDelta);
        assertEquals(0, r.getZ(), kDelta);
    }

    @Test
    public void testPitchMatrix() {
        // in FRC Y is the pitch axis, positive pitch is clockwise looking in the Y
        // direction, which means pitch down.
        Matrix<N3, N3> matrix = new Matrix<>(Nat.N3(), Nat.N3());
        double rot = Math.sqrt(2) / 2;
        /*
         * [[0.7, 0.0, 0.7],
         * [0.0, 1.0, 0.0],
         * [-0.7, 0.0, 0.7]]
         */
        matrix.set(0, 0, rot);
        matrix.set(0, 1, 0);
        matrix.set(0, 2, rot);

        matrix.set(1, 0, 0);
        matrix.set(1, 1, 1.0);
        matrix.set(1, 2, 0);

        matrix.set(2, 0, -rot);
        matrix.set(2, 1, 0);
        matrix.set(2, 2, rot);

        Rotation3d r = new Rotation3d(matrix);
        assertEquals(0, r.getX(), kDelta);
        assertEquals(Math.PI / 4, r.getY(), kDelta);
        assertEquals(0, r.getZ(), kDelta);
    }

    @Test
    public void testYawMatrix() {
        // in FRC Z is the yaw axis, positive yaw is clockwise looking in the Z
        // direction, which means yaw left.
        Matrix<N3, N3> matrix = new Matrix<>(Nat.N3(), Nat.N3());
        double rot = Math.sqrt(2) / 2;
        /*
         * [[0.7, -0.7, 0.0],
         * [0.7, 0.7, 0.0],
         * [0.0, 0.0, 1.0]]
         */
        matrix.set(0, 0, rot);
        matrix.set(0, 1, -rot);
        matrix.set(0, 2, 0);

        matrix.set(1, 0, rot);
        matrix.set(1, 1, rot);
        matrix.set(1, 2, 0);

        matrix.set(2, 0, 0);
        matrix.set(2, 1, 0);
        matrix.set(2, 2, 1.0);

        Rotation3d r = new Rotation3d(matrix);
        assertEquals(0, r.getX(), kDelta);
        assertEquals(0, r.getY(), kDelta);
        assertEquals(Math.PI / 4, r.getZ(), kDelta);
    }
}
