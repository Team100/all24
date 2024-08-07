package org.team100.math;

import java.util.List;
import java.util.function.DoubleUnaryOperator;
import java.util.function.Function;
import java.util.stream.DoubleStream;
import java.util.stream.Stream;

import edu.wpi.first.math.jni.EigenJNI;

public class RBFInterpolatingFunction<T, U> implements Function<T, U> {
    public static final DoubleUnaryOperator GAUSSIAN = r -> Math.exp(-1.0 * Math.pow(r, 2));

    abstract static class Adapter<V> {
        abstract int size();

        abstract double[] toArray(V src);

        abstract V fromArray(double[] src);

        /** Converts 2d array to 1d row-major array. */
        double[] rowMajor(List<V> x) {
            if (x.isEmpty())
                return new double[0];
            final int rows = x.size();
            final int cols = size();
            final double[] result = new double[rows * cols];
            for (int row = 0; row < rows; ++row) {
                final double[] rowdata = toArray(x.get(row));
                System.arraycopy(rowdata, 0, result, row * cols, cols);
            }
            return result;
        }

        /** Distance metric in the space of V. */
        double r(V a, V b) {
            double ss = 0.0;
            final double[] adata = toArray(a);
            final double[] bdata = toArray(b);
            for (int i = 0; i < size(); ++i) {
                ss += Math.pow(adata[i] - bdata[i], 2);
            }
            return Math.sqrt(ss);
        }
    }

    final List<T> m_x;
    final List<U> m_y;
    final Adapter<T> m_xadapter;
    final Adapter<U> m_yadapter;
    final DoubleUnaryOperator m_rbf;
    final double[][] m_w;

    /** x and y must be parallel. */
    public RBFInterpolatingFunction(
            List<T> x,
            List<U> y,
            Adapter<T> xadapter,
            Adapter<U> yadapter,
            DoubleUnaryOperator rbf) {
        if (x.isEmpty() || y.isEmpty())
            throw new IllegalArgumentException();
        if (x.size() != y.size())
            throw new IllegalArgumentException();
        m_x = x;
        m_y = y;
        m_xadapter = xadapter;
        m_yadapter = yadapter;
        m_rbf = rbf;

        int xRows = m_x.size();
        int yRows = m_y.size();
        int yCols = yadapter.size();
        double[] yRowMajor = yadapter.rowMajor(m_y);

        double[][] phi = phi(m_x, m_xadapter, m_rbf);
        int phiRows = xRows;
        int phiCols = xRows;
        double[] phiRowMajor = rowMajor(phi);

        int wRows = yRows;
        int wCols = yCols;
        double[] wRowMajor = new double[yRowMajor.length];
        EigenJNI.solveFullPivHouseholderQr(phiRowMajor, phiRows, phiCols, yRowMajor, yRows, yCols, wRowMajor);
        m_w = twoD(wRowMajor, wRows, wCols);
    }

    @Override
    public U apply(T t) {
        double[] phiVec = phiVec(t, m_x, m_xadapter, m_rbf);
        double[] result = new double[m_yadapter.size()];
        for (int i = 0; i < m_yadapter.size(); ++i) {
            for (int j = 0; j < phiVec.length; ++j) {
                result[i] += phiVec[j] * m_w[j][i];
            }
        }
        return m_yadapter.fromArray(result);
    }

    /** Returns basis function evaluated for each pair in x. */
    static <T> double[][] phi(List<T> x, Adapter<T> xadapter, DoubleUnaryOperator rbf) {
        double[][] phi = new double[x.size()][x.size()];
        for (int i = 0; i < x.size(); ++i) {
            for (int j = 0; j < x.size(); ++j) {
                phi[i][j] = rbf.applyAsDouble(xadapter.r(x.get(i), x.get(j)));
            }
        }
        return phi;
    }

    static <T> double[] phiVec(T p, List<T> x, Adapter<T> xadapter, DoubleUnaryOperator rbf) {
        double[] result = new double[x.size()];
        for (int i = 0; i < x.size(); ++i) {
            result[i] = rbf.applyAsDouble(xadapter.r(p, x.get(i)));
        }
        return result;
    }

    /** Converts 2d array to 1d row-major array. */
    static double[] rowMajor(double[][] x) {
        return Stream.of(x)
                .flatMapToDouble(DoubleStream::of)
                .toArray();
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
