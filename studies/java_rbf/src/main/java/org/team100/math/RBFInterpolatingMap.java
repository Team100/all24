package org.team100.math;

import java.util.ArrayList;
import java.util.List;
import java.util.function.DoubleUnaryOperator;
import java.util.stream.DoubleStream;
import java.util.stream.Stream;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Num;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.jni.EigenJNI;

/**
 * @param X independent dimensionality
 * @param Y dependent dimensionality
 */
public class RBFInterpolatingMap<X extends Num, Y extends Num> {
    /** Radial basis function */
    final DoubleUnaryOperator m_rbf;
    /** Independent variable, X dimensionality */
    final List<Vector<X>> m_x;
    /** Dependent variable, Y dimensionality */
    final List<Vector<Y>> m_y;
    /** Weights learned from the lists above */
    double[][] m_w;

    public RBFInterpolatingMap(DoubleUnaryOperator rbf) {
        m_rbf = rbf;
        m_x = new ArrayList<>();
        m_y = new ArrayList<>();
    }

    public void put(Vector<X> x, Vector<Y> y) {
        m_x.add(x);
        m_y.add(y);
        m_w = null;
    }

    public Vector<Y> get(Vector<X> x) {
        // if not up to date, calculate the weights

    }

    void calculateWeights() {

        int xRows = x.length;
        int xCols = x[0].length;
        double[] xRowMajor = rowMajor(m_x);

        int yRows = y.length;
        int yCols = y[0].length;
        double[] yRowMajor = rowMajor(m_y);

        double[][] phi = phi(m_x, m_rbf);
        int phiRows = xRows;
        int phiCols = xRows;
        double[] phiRowMajor = rowMajor(phi);

        int wRows = yRows;
        int wCols = yCols;
        double[] wRowMajor = new double[yRowMajor.length];
        EigenJNI.solveFullPivHouseholderQr(phiRowMajor, phiRows, phiCols, yRowMajor, yRows, yCols, wRowMajor);
        m_w = twoD(wRowMajor, wRows, wCols);
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
    static <T extends Num> double[] rowMajor(List<Vector<T>> x) {
        if (x.isEmpty())
            return new double[0];

        int cols = x.get(0).getNumRows();
        int rows = x.size();
        double[] result = new double[rows * cols];
        for (int row = 0; row < rows; ++row) {
            for (int col = 0; col < cols; ++col) {
                result[row * cols + col] = x.get(row).get(col);
            }
        }
        return result;
    }

    /** Convert a row-major array into a 2d array. */
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
