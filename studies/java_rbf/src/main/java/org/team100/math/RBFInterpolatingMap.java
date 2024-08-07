package org.team100.math;

import java.util.ArrayList;
import java.util.List;
import java.util.function.DoubleUnaryOperator;
import java.util.stream.DoubleStream;
import java.util.stream.Stream;

import edu.wpi.first.math.Nat;
import edu.wpi.first.math.Num;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.jni.EigenJNI;

/**
 * @param X independent dimensionality
 * @param Y dependent dimensionality
 */
public class RBFInterpolatingMap<X extends Num, Y extends Num> {
    public static final DoubleUnaryOperator GAUSSIAN = r -> Math.exp(-1.0 * Math.pow(r, 2));

    /** Radial basis function */
    final DoubleUnaryOperator m_rbf;
    /** Independent variable, X dimensionality */
    final List<Vector<X>> m_x;
    /** Dependent variable, Y dimensionality */
    final List<Vector<Y>> m_y;
    final Nat<Y> m_instance;
    /** Weights learned from the lists above */
    double[][] m_w;

    public RBFInterpolatingMap(DoubleUnaryOperator rbf, Nat<Y> instance) {
        m_rbf = rbf;
        m_x = new ArrayList<>();
        m_y = new ArrayList<>();
        m_instance = instance;
    }

    public void put(Vector<X> x, Vector<Y> y) {
        m_x.add(x);
        m_y.add(y);
        m_w = null;
    }

    public Vector<Y> get(Vector<X> p) {
        // if not up to date, calculate the weights
        if (m_w == null)
            calculateWeights();
        if (m_x.isEmpty())
            return new Vector<>(m_instance);
        if (p.getNumRows() != m_x.get(0).getNumRows())
            throw new IllegalArgumentException();
        double[] phiVec = phiVec(p, m_x, m_rbf);
        Vector<Y> result = new Vector<>(m_instance);
        for (int i = 0; i < result.getNumRows(); ++i) {
            double cell = 0.0;
            for (int j = 0; j < phiVec.length; ++j) {
                cell += phiVec[j] * m_w[j][i];
            }
            result.set(i, 0, cell);
        }
        return result;

    }

    void calculateWeights() {

        int xRows = m_x.size();
        int xCols = m_x.get(0).getNumRows();
        double[] xRowMajor = rowMajor(m_x);

        int yRows = m_y.size();
        int yCols = m_y.get(0).getNumRows();
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
    static <T extends Num> double[][] phi(List<Vector<T>> x, DoubleUnaryOperator rbf) {
        double[][] phi = new double[x.size()][x.size()];
        for (int i = 0; i < x.size(); ++i) {
            for (int j = 0; j < x.size(); ++j) {
                phi[i][j] = rbf.applyAsDouble(r(x.get(i), x.get(j)));
            }
        }
        return phi;
    }

    static <T extends Num> double[] phiVec(Vector<T> p, List<Vector<T>> x, DoubleUnaryOperator rbf) {
        double[] result = new double[x.size()];
        for (int i = 0; i < x.size(); ++i) {
            result[i] = rbf.applyAsDouble(r(p, x.get(i)));
        }
        return result;
    }

    /** Returns Euclidean distance from a to b. */
    static <T extends Num> double r(Vector<T> a, Vector<T> b) {
        if (a.getNumRows() != b.getNumRows())
            throw new IllegalArgumentException();
        double ss = 0.0;
        for (int i = 0; i < a.getNumRows(); ++i) {
            ss += Math.pow(a.get(i) - b.get(i), 2);
        }
        return Math.sqrt(ss);
    }

    /** Converts 2d array to 1d row-major array. */
    static double[] rowMajor(double[][] x) {
        return Stream.of(x)
                .flatMapToDouble(DoubleStream::of)
                .toArray();
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
