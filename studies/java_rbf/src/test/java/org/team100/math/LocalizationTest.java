package org.team100.math;

import java.io.BufferedReader;
import java.io.FileReader;
import java.io.IOException;
import java.nio.file.Paths;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

import org.junit.jupiter.api.Test;

class LocalizationTest {
    record uvdZ(double u, double v, double d, double Z) {
        @Override
        public String toString() {
            return String.format("[%.2f, %.2f, %.2f, %.2f]", u, v, d, Z);
        }
    }

    record XY(double X, double Y) {
        @Override
        public String toString() {
            return String.format("[%.2f, %.2f]", X, Y);
        }
    }

    static class uvdZAdapter extends RBFInterpolatingFunction.Adapter<uvdZ> {

        @Override
        int size() {
            return 4;
        }

        @Override
        double[] toArray(uvdZ src) {
            return new double[] { src.u, src.v, src.d, src.Z };
        }

        @Override
        uvdZ fromArray(double[] src) {
            return new uvdZ(src[0], src[1], src[2], src[3]);
        }

    }

    static class XYAdapter extends RBFInterpolatingFunction.Adapter<XY> {

        @Override
        int size() {
            return 2;
        }

        @Override
        double[] toArray(XY src) {
            return new double[] { src.X, src.Y };
        }

        @Override
        XY fromArray(double[] src) {
            return new XY(src[0], src[1]);
        }

    }

    static uvdZAdapter xAdapter = new uvdZAdapter();
    static XYAdapter yAdapter = new XYAdapter();

    /**
     * This test shows that we need to normalize the model. Some of the input
     * dimensions cover hundreds of units, others cover less than one, but the
     * "radius" in the radial basis function treats all these dimensions the same,
     * with its simple Euclidean norm.
     */
    @Test
    void testUvdZXY() throws IOException {

        System.out.println("PATH " + Paths.get(".").toAbsolutePath().normalize().toString());

        double[][] uvdZXY_train = readfile("../../../uvdZXY_train.csv");
        double[][] uvdZXY_test = readfile("../../../uvdZXY_test.csv");

        // independent variables
        List<uvdZ> uvdZ_train = getuvdZ(uvdZXY_train);
        List<uvdZ> uvdZ_test = getuvdZ(uvdZXY_test);
        // dependent variables
        List<XY> XY_train = getXY(uvdZXY_train);
        List<XY> XY_test = getXY(uvdZXY_test);

        for (double vari = 1; vari < 10; vari *= 1.05) {
            final double variance = vari;
            RBFInterpolatingFunction<uvdZ, XY> rbf = new RBFInterpolatingFunction<>(
                    uvdZ_train,
                    XY_train,
                    xAdapter,
                    yAdapter,
                    RBFInterpolatingFunction.GAUSSIAN(variance));

            double errSum = 0;
            for (int i = 0; i < XY_test.size(); ++i) {
                XY predictedXY = rbf.apply(uvdZ_test.get(i));

                double errX = predictedXY.X - XY_test.get(i).X;
                double errY = predictedXY.Y - XY_test.get(i).Y;

                double err = Math.hypot(errX, errY);
                errSum += err;
                // System.out.printf("X %6.3f predictedX %6.3f errX %6.3f Y %6.3f predictedY
                // %6.3f errY %6.3f\n",
                // XY_test.get(i).X, predictedXY.X, errX, XY_test.get(i).Y, predictedXY.Y,
                // errY);
            }
            double mae = errSum / XY_test.size();
            System.out.printf("variance %5.3f mae %5.3f\n", variance, mae);
        }
    }

    @Test
    void testUvdZXYBestVariance() throws IOException {

        System.out.println("PATH " + Paths.get(".").toAbsolutePath().normalize().toString());

        double[][] uvdZXY_train = readfile("../../../uvdZXY_train.csv");
        double[][] uvdZXY_test = readfile("../../../uvdZXY_test.csv");

        // independent variables
        List<uvdZ> uvdZ_train = getuvdZ(uvdZXY_train);
        List<uvdZ> uvdZ_test = getuvdZ(uvdZXY_test);
        // dependent variables
        List<XY> XY_train = getXY(uvdZXY_train);
        List<XY> XY_test = getXY(uvdZXY_test);

        final double variance = 3.0;
        RBFInterpolatingFunction<uvdZ, XY> rbf = new RBFInterpolatingFunction<>(
                uvdZ_train,
                XY_train,
                xAdapter,
                yAdapter,
                RBFInterpolatingFunction.GAUSSIAN(variance));

        double errSum = 0;
        for (int i = 0; i < XY_test.size(); ++i) {
            XY predictedXY = rbf.apply(uvdZ_test.get(i));

            double errX = predictedXY.X - XY_test.get(i).X;
            double errY = predictedXY.Y - XY_test.get(i).Y;

            double err = Math.hypot(errX, errY);
            errSum += err;
            System.out.printf("X %6.3f predictedX %6.3f errX %6.3f Y %6.3f predictedY %6.3f errY %6.3f\n",
                    XY_test.get(i).X, predictedXY.X, errX, XY_test.get(i).Y, predictedXY.Y,
                    errY);
        }
        double mae = errSum / XY_test.size();
        System.out.printf("variance %5.3f mae %5.3f\n", variance, mae);

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

    private static List<uvdZ> getuvdZ(double[][] uvdZXY) {
        List<uvdZ> result = new ArrayList<>();
        for (int i = 0; i < uvdZXY.length; ++i) {
            double[] src = uvdZXY[i];
            uvdZ e = new uvdZ(src[0], src[1], src[2], src[3]);
            result.add(e);
        }
        return result;
    }

    private static List<XY> getXY(double[][] uvdZXY) {
        List<XY> result = new ArrayList<>();
        for (int i = 0; i < uvdZXY.length; ++i) {
            double[] src = uvdZXY[i];
            XY e = new XY(src[4], src[5]);
            result.add(e);
        }
        return result;
    }

}
