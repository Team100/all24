package org.team100.math;

import java.io.BufferedReader;
import java.io.FileReader;
import java.io.IOException;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

import org.opencv.calib3d.Calib3d;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.MatOfDouble;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.MatOfPoint3f;
import org.opencv.core.Point3;

import smile.interpolation.RBFInterpolation;
import smile.math.rbf.GaussianRadialBasis;

// see accuracy.ipynb
public class Accuracy {

    public void run() {

        MatOfPoint3f objectPoints = new MatOfPoint3f(
                new Point3(0.0, 1.0, 1.0),
                new Point3(0.0, 1.0, 5.0),
                new Point3(0.0, 1.0, 10.0));
        Mat rvec = new MatOfDouble(-0.4, 0.0, 0.0);
        Mat tvec = new MatOfDouble(0.0, 0.2, 0.0);
        Mat cameraMatrix = Mat.zeros(3, 3, CvType.CV_64F);
        cameraMatrix.put(0, 0,
                300.0, 0.0, 250.0,
                0.0, 300.0, 250.0,
                0.0, 0.0, 1.0);
        MatOfDouble distCoeffs = new MatOfDouble(1.5, -0.95, -0.005, 0.0025, 1.16);
        MatOfPoint2f imagePoints = new MatOfPoint2f();

        Calib3d.projectPoints(objectPoints, rvec, tvec, cameraMatrix, distCoeffs, imagePoints);

        System.out.println(imagePoints);

    }

    public static void main(String[] args) throws IOException {
        System.out.println("hello");
        double[][] uvdZXY_train = readfile("uvdZXY_train.csv");
        double[][] uvdZXY_test = readfile("uvdZXY_test.csv");

        // independent variables
        double[][] uvdZ_train = getuvdZ(uvdZXY_train);
        double[][] uvdZ_test = getuvdZ(uvdZXY_test);
        // dependent variables
        double[] X_train = getX(uvdZXY_train);
        double[] X_test = getX(uvdZXY_test);
        double[] Y_train = getY(uvdZXY_train);
        double[] Y_test = getY(uvdZXY_test);

        RBFInterpolation rbfX = new RBFInterpolation(
                uvdZ_train,
                X_train,
                new GaussianRadialBasis(15));
        RBFInterpolation rbfY = new RBFInterpolation(
                uvdZ_train,
                Y_train,
                new GaussianRadialBasis(15));

        double errSum = 0;
        for (int i = 0; i < X_test.length; ++i) {
            double predictedX = rbfX.interpolate(uvdZ_test[i]);
            double predictedY = rbfY.interpolate(uvdZ_test[i]);

            double errX = predictedX - X_test[i];
            double errY = predictedY - Y_test[i];

            double err = Math.hypot(errX, errY);
            errSum += err;
            System.out.printf("X %6.3f predictedX %6.3f Y %6.3f predictedY %6.3f\n",
                    X_test[i], predictedX, Y_test[i], predictedY);
        }
        double mae = errSum / X_test.length;
        System.out.printf("mae %5.3f\n", mae);

    }

    private static double[][] readfile(String filename) throws IOException {
        try (BufferedReader br = new BufferedReader(new FileReader(filename))) {
            String line;
            List<double[]> rows = new ArrayList<>();
            while ((line = br.readLine()) != null) {
                String[] values = line.split(" ");
                double[] dvals = Arrays.stream(values).mapToDouble(Double::parseDouble).toArray();
                rows.add(dvals);
            }
            double[][] result = rows.stream().toArray(double[][]::new);

            System.out.printf("%s rows: %d\n", filename, result.length);
            System.out.printf("%s columns: %d\n", filename, result[0].length);
            return result;
        }
    }

    private static double[][] getuvdZ(double[][] uvdZXY) {
        double[][] uvdZ = new double[uvdZXY.length][4];
        for (int i = 0; i < uvdZXY.length; ++i) {
            uvdZ[i][0] = uvdZXY[i][0];
            uvdZ[i][1] = uvdZXY[i][1];
            uvdZ[i][2] = uvdZXY[i][2];
            uvdZ[i][3] = uvdZXY[i][3];
        }
        return uvdZ;
    }

    private static double[] getX(double[][] uvdZXY) {
        double[] X = new double[uvdZXY.length];
        for (int i = 0; i < uvdZXY.length; ++i) {
            X[i] = uvdZXY[i][4];
        }
        return X;
    }

    private static double[] getY(double[][] uvdZXY) {
        double[] Y = new double[uvdZXY.length];
        for (int i = 0; i < uvdZXY.length; ++i) {
            Y[i] = uvdZXY[i][5];
        }
        return Y;
    }
}
