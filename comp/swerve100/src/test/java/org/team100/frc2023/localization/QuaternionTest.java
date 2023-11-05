package org.team100.frc2023.localization;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertThrows;

import java.io.IOException;

import org.junit.jupiter.api.Test;
import org.opencv.calib3d.Calib3d;
import org.opencv.core.CvType;
import org.opencv.core.Mat;

import edu.wpi.first.cscore.CameraServerCvJNI;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.numbers.N3;

/**
 * Figure out if renormalization works.
 * 
 * Note since we don't trust the camera rotation output we really don't need to
 * do this.
 */
class QuaternionTest {
    private static final double kDelta = 0.01;

    QuaternionTest() throws IOException {
        // load the JNI
        CameraServerCvJNI.forceLoad();
    }

    @Test
    void testRodriguesIdentity() {
        // just to see if it works at all, this is no rotation.
        Mat rmat = new Mat(3, 3, CvType.CV_32F);
        rmat.put(0, 0, 1);
        rmat.put(0, 1, 0);
        rmat.put(0, 2, 0);

        rmat.put(1, 0, 0);
        rmat.put(1, 1, 1);
        rmat.put(1, 2, 0);

        rmat.put(2, 0, 0);
        rmat.put(2, 1, 0);
        rmat.put(2, 2, 1);

        // convert it to axis-angle
        Mat rvec = new Mat(3, 1, CvType.CV_32F);
        Calib3d.Rodrigues(rmat, rvec);

        // i imagine this is special-cased
        assertEquals(0, rvec.get(0, 0)[0], kDelta);
        assertEquals(0, rvec.get(1, 0)[0], kDelta);
        assertEquals(0, rvec.get(2, 0)[0], kDelta);

        // convert back to rotation matrix -- this should be orthogonal
        Mat rmat2 = new Mat();
        Calib3d.Rodrigues(rvec, rmat2);

        assertEquals(1, rmat2.get(0, 0)[0], kDelta);
        assertEquals(0, rmat2.get(0, 1)[0], kDelta);
        assertEquals(0, rmat2.get(0, 2)[0], kDelta);

        assertEquals(0, rmat2.get(1, 0)[0], kDelta);
        assertEquals(1, rmat2.get(1, 1)[0], kDelta);
        assertEquals(0, rmat2.get(1, 2)[0], kDelta);

        assertEquals(0, rmat2.get(2, 0)[0], kDelta);
        assertEquals(0, rmat2.get(2, 1)[0], kDelta);
        assertEquals(1, rmat2.get(2, 2)[0], kDelta);

        Matrix<N3, N3> matrix = new Matrix<>(Nat.N3(), Nat.N3());
        for (int row = 0; row < 3; ++row) {
            for (int col = 0; col < 3; ++col) {
                matrix.set(row, col, rmat2.get(row, col)[0]);
            }
        }

        // this should never fail
        Rotation3d r = new Rotation3d(matrix);
        assertEquals(0, r.getX(), kDelta);
        assertEquals(0, r.getY(), kDelta);
        assertEquals(0, r.getZ(), kDelta);
    }

    @Test
    void testRodriguesSimple() {
        // a simple rotation
        Mat rmat = new Mat(3, 3, CvType.CV_64F);
        double rot = Math.sqrt(2) / 2;
        rmat.put(0, 0, 1);
        rmat.put(0, 1, 0);
        rmat.put(0, 2, 0);

        rmat.put(1, 0, 0);
        rmat.put(1, 1, rot);
        rmat.put(1, 2, -rot);

        rmat.put(2, 0, 0);
        rmat.put(2, 1, rot);
        rmat.put(2, 2, rot);

        // convert it to axis-angle
        Mat rvec = new Mat(3, 1, CvType.CV_64F); // must be 64 bit to pass rotation3d orthogonality constraint :-(
        Calib3d.Rodrigues(rmat, rvec);

        assertEquals(Math.PI / 4, rvec.get(0, 0)[0], kDelta);
        assertEquals(0, rvec.get(1, 0)[0], kDelta);
        assertEquals(0, rvec.get(2, 0)[0], kDelta);

        // convert back to rotation matrix -- this should be orthogonal
        Mat rmat2 = new Mat();
        Calib3d.Rodrigues(rvec, rmat2);

        assertEquals(1, rmat2.get(0, 0)[0], kDelta);
        assertEquals(0, rmat2.get(0, 1)[0], kDelta);
        assertEquals(0, rmat2.get(0, 2)[0], kDelta);

        assertEquals(0, rmat2.get(1, 0)[0], kDelta);
        assertEquals(rot, rmat2.get(1, 1)[0], kDelta);
        assertEquals(-rot, rmat2.get(1, 2)[0], kDelta);

        assertEquals(0, rmat2.get(2, 0)[0], kDelta);
        assertEquals(rot, rmat2.get(2, 1)[0], kDelta);
        assertEquals(rot, rmat2.get(2, 2)[0], kDelta);

        Matrix<N3, N3> matrix = new Matrix<>(Nat.N3(), Nat.N3());
        for (int row = 0; row < 3; ++row) {
            for (int col = 0; col < 3; ++col) {
                matrix.set(row, col, rmat2.get(row, col)[0]);
            }
        }

        // this is what Rotation3d checks
        assertEquals(0, matrix.times(matrix.transpose()).minus(Matrix.eye(Nat.N3())).normF(), 1e-9);

        // this should never fail
        Rotation3d r = new Rotation3d(matrix);
        assertEquals(Math.PI / 4, r.getX(), kDelta);
        assertEquals(0, r.getY(), kDelta);
        assertEquals(0, r.getZ(), kDelta);
    }

    @Test
    void testRodriguesNonorthogonal() {
        // a nonorthogonal matrix, this should fail
        Mat rmat = new Mat(3, 3, CvType.CV_32F);
        double rot = Math.sqrt(2) / 2;
        rmat.put(0, 0, 1);
        rmat.put(0, 1, 0);
        rmat.put(0, 2, 0);

        rmat.put(1, 0, 0);
        rmat.put(1, 1, rot * 0.99); // error
        rmat.put(1, 2, -rot);

        rmat.put(2, 0, 0);
        rmat.put(2, 1, rot);
        rmat.put(2, 2, rot);

        Matrix<N3, N3> matrix = new Matrix<>(Nat.N3(), Nat.N3());
        for (int row = 0; row < 3; ++row) {
            for (int col = 0; col < 3; ++col) {
                matrix.set(row, col, rmat.get(row, col)[0]);
            }
        }

        // this is way over the limit of 1e-9
        assertEquals(0.012, matrix.times(matrix.transpose()).minus(Matrix.eye(Nat.N3())).normF(), kDelta);

        // this should fail because the matrix is not orthogonal
        // this puts the stack trace on stderr and there's no easy way to make it quiet.
        assertThrows(IllegalArgumentException.class, () -> {
            new Rotation3d(matrix);
        });

    }

    @Test
    void testRodriguesNormalized() {
        // a simple rotation
        Mat rmat = new Mat(3, 3, CvType.CV_64F); // must use 64 bit
        double rot = Math.sqrt(2) / 2;
        rmat.put(0, 0, 1);
        rmat.put(0, 1, 0);
        rmat.put(0, 2, 0);

        rmat.put(1, 0, 0);
        rmat.put(1, 1, rot * 0.99); // same error as above
        rmat.put(1, 2, -rot);

        rmat.put(2, 0, 0);
        rmat.put(2, 1, rot);
        rmat.put(2, 2, rot);

        // verify that it's bad
        Matrix<N3, N3> matrix1 = new Matrix<>(Nat.N3(), Nat.N3());
        for (int row = 0; row < 3; ++row) {
            for (int col = 0; col < 3; ++col) {
                matrix1.set(row, col, rmat.get(row, col)[0]);
            }
        }

        // this is way over the limit of 1e-9
        assertEquals(0.012, matrix1.times(matrix1.transpose()).minus(Matrix.eye(Nat.N3())).normF(), kDelta);

        // convert it to axis-angle
        Mat rvec = new Mat(3, 1, CvType.CV_64F); // must use 64 bit
        Calib3d.Rodrigues(rmat, rvec);

        assertEquals(Math.PI / 4, rvec.get(0, 0)[0], kDelta);
        assertEquals(0, rvec.get(1, 0)[0], kDelta);
        assertEquals(0, rvec.get(2, 0)[0], kDelta);

        // convert back to rotation matrix -- this should be orthogonal
        Mat rmat2 = new Mat();
        Calib3d.Rodrigues(rvec, rmat2);

        assertEquals(1, rmat2.get(0, 0)[0], kDelta);
        assertEquals(0, rmat2.get(0, 1)[0], kDelta);
        assertEquals(0, rmat2.get(0, 2)[0], kDelta);

        assertEquals(0, rmat2.get(1, 0)[0], kDelta);
        assertEquals(rot, rmat2.get(1, 1)[0], kDelta);
        assertEquals(-rot, rmat2.get(1, 2)[0], kDelta);

        assertEquals(0, rmat2.get(2, 0)[0], kDelta);
        assertEquals(rot, rmat2.get(2, 1)[0], kDelta);
        assertEquals(rot, rmat2.get(2, 2)[0], kDelta);

        Matrix<N3, N3> matrix = new Matrix<>(Nat.N3(), Nat.N3());
        for (int row = 0; row < 3; ++row) {
            for (int col = 0; col < 3; ++col) {
                matrix.set(row, col, rmat2.get(row, col)[0]);
            }
        }

        // this is what Rotation3d checks
        assertEquals(0, matrix.times(matrix.transpose()).minus(Matrix.eye(Nat.N3())).normF(), 1e-9);

        // this should never fail
        Rotation3d r = new Rotation3d(matrix);
        assertEquals(Math.PI / 4, r.getX(), kDelta);
        assertEquals(0, r.getY(), kDelta);
        assertEquals(0, r.getZ(), kDelta);
    }
}
