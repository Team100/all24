package org.team100.math;

import java.util.function.DoubleUnaryOperator;
import java.util.stream.DoubleStream;
import java.util.stream.Stream;

import edu.wpi.first.math.jni.EigenJNI;

public class RBFInterpolator {

    public static final DoubleUnaryOperator GAUSSIAN = r -> Math.exp(-1.0 * Math.pow(r, 2));
    /**
     * Training independent vars, i.e. points where rbf is evaluated later. One row
     * per observation, one column per input dimension.
     */
    final double[][] m_x;
    /**
     * The weights learned below. One row per observation, one column per output
     * dimension
     */
    final double[][] m_w;
    final DoubleUnaryOperator m_rbf;

    /**
     * Solves $\Phi W = F$
     * 
     * @param x   known independent variables, one row per observation, one column
     *            per variable
     * @param y   known dependent variables at those points, one row per
     *            observation, one column per variable.
     * @param rbf radial basis function
     */
    public RBFInterpolator(double[][] x, double[][] y, DoubleUnaryOperator rbf) {
        if (x.length != y.length)
            throw new IllegalArgumentException();
        m_x = x;
        m_rbf = rbf;

        int xRows = x.length;
        int xCols = x[0].length;
        double[] xRowMajor = rowMajor(x);

        int yRows = y.length;
        int yCols = y[0].length;
        double[] yRowMajor = rowMajor(y);

        double[][] phi = phi(x, rbf);
        int phiRows = xRows;
        int phiCols = xRows;
        double[] phiRowMajor = rowMajor(phi);

        int wRows = yRows;
        int wCols = yCols;
        double[] wRowMajor = new double[yRowMajor.length];
        EigenJNI.solveFullPivHouseholderQr(phiRowMajor, phiRows, phiCols, yRowMajor, yRows, yCols, wRowMajor);
        m_w = twoD(wRowMajor, wRows, wCols);
    }

    /**
     * Interpolate for one location.
     * 
     * for this we evaluate the rbfs using each of the known positions and probe
     * position,
     * (remember the rbf is a scalar function based on the distance to each known
     * position)
     * and then multiply by the weights we learned in the constructor
     */
    public double[] get(double[] p) {
        if (p.length != m_x[0].length)
            throw new IllegalArgumentException();
        double[] phiVec = phiVec(p, m_x, m_rbf);
        double[] result = new double[m_w[0].length];
        for (int i = 0; i < result.length; ++i) {
            result[i] = 0.0;
            for (int j = 0; j < phiVec.length; ++j) {
                result[i] += phiVec[j] * m_w[j][i];
            }
        }
        return result;
    }

    /** Returns basis function evaluated for each pair in x. */
    static double[][] phi(double[][] x, DoubleUnaryOperator rbf) {
        double[][] phi = new double[x.length][x.length];
        for (int i = 0; i < x.length; ++i) {
            for (int j = 0; j < x.length; ++j) {
                phi[i][j] = rbf.applyAsDouble(r(x[i], x[j]));
            }
        }
        return phi;
    }

    /**
     * 
     * @param p   point we want
     * @param x   known points
     * @param rbf
     * @return
     */
    static double[] phiVec(double[] p, double[][] x, DoubleUnaryOperator rbf) {
        double[] result = new double[x.length];
        for (int i = 0; i < x.length; ++i) {
            result[i] = rbf.applyAsDouble(r(p, x[i]));
        }
        return result;
    }

    /** Returns Euclidean distance from a to b. */
    static double r(double[] a, double[] b) {
        if (a.length != b.length)
            throw new IllegalArgumentException();
        double ss = 0.0;
        for (int i = 0; i < a.length; ++i) {
            ss += Math.pow(a[i] - b[i], 2);
        }
        return Math.sqrt(ss);
    }

    /** Converts 2d array to 1d row-major array. */
    static double[] rowMajor(double[][] x) {
        return Stream.of(x)
                .flatMapToDouble(DoubleStream::of)
                .toArray();
    }

    static double[][] twoD(double[] x, int rows, int cols) {
        if (x.length != rows * cols)
            throw new IllegalArgumentException();
        double[][] result = new double[rows][cols];
        for (int row = 0; row < rows; row++) {
            for (int col = 0; col < cols; col++) {
                result[row][col] = x[row * cols + col];
            }
        }
        return result;
    }

}
