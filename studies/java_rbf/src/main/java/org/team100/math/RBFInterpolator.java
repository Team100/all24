package org.team100.math;

import java.util.function.DoubleUnaryOperator;
import java.util.stream.DoubleStream;
import java.util.stream.Stream;

public class RBFInterpolator {

    public static final DoubleUnaryOperator GAUSSIAN = r -> Math.exp(-1.0 * Math.pow(r, 2));

    /**
     * 
     * @param x   known independent variables
     * @param y   known dependent variables at those points
     * @param rbf radial basis function
     */
    public RBFInterpolator(double[][] x, double[][] y, DoubleUnaryOperator rbf) {

        double[] xRowMajor = rowMajor(x);
        double[] yRowMajor = rowMajor(y);

        for (double xx : xRowMajor) {
            System.out.println(xx);
        }
        for (double yy : yRowMajor) {
            System.out.println(yy);
        }

    }

    static double[][] phi(double[][] x, DoubleUnaryOperator rbf) {
        double[][] phi = new double[x.length][x.length];
        for (int i = 0; i < x.length; ++i) {
            for (int j = 0; j < x.length; ++j) {
                phi[i][j] = rbf.applyAsDouble(r(x[i], x[j]));
            }
        }
        return phi;
    }

    /** Euclidean distance from a to b. */
    static double r(double[] a, double[] b) {
        if (a.length != b.length)
            throw new IllegalArgumentException();
        double ss = 0.0;
        for (int i = 0; i < a.length; ++i) {
            ss += Math.pow(a[i] - b[i], 2);
        }
        return Math.sqrt(ss);
    }

    private double[] rowMajor(double[][] x) {
        return Stream.of(x)
                .flatMapToDouble(DoubleStream::of)
                .toArray();
    }

}
