package org.team100.math;

import java.util.List;
import java.util.function.DoubleUnaryOperator;
import java.util.function.Function;
import java.util.stream.DoubleStream;
import java.util.stream.Stream;

import edu.wpi.first.math.jni.EigenJNI;

public class RBFInterpolatingFunction<T, U> implements Function<T, U> {
    // prevent division by zero
    private static final double kMinVariance = 0.01;

    /** Produces a gaussian scaled to 1 at 0. */
    public static DoubleUnaryOperator GAUSSIAN(double variance) {
        return r -> Math.exp(-1.0 * Math.pow(r, 2) / variance) / variance;
    }

    record Stats(double mean, double stddev) {
    }

    abstract static class Adapter<V> {
        abstract int size();

        abstract double[] toArray(V src);

        abstract V fromArray(double[] src);

        Stats[] stats(List<V> x) {
            double[] sum = new double[size()];
            double[] sq_sum = new double[size()];
            for (V v : x) {
                double[] a = toArray(v);
                for (int i = 0; i < size(); ++i) {
                    sum[i] += a[i];
                    sq_sum[i] += a[i] * a[i];
                }
            }
            Stats[] stats = new Stats[size()];
            for (int i = 0; i < size(); ++i) {
                double mean = sum[i] / x.size();
                double variance = sq_sum[i] / x.size() - mean * mean;
                variance = Math.max(variance, kMinVariance);
                double stddev = Math.sqrt(variance);
                stats[i] = new Stats(mean, stddev);
            }
            return stats;
        }

        /** Converts 2d array to a normalized 1d row-major array. */
        double[] rowMajor(List<V> x, Stats[] stats) {
            if (x.isEmpty())
                return new double[0];
            final int rows = x.size();
            final int cols = size();
            final double[] result = new double[rows * cols];
            for (int row = 0; row < rows; ++row) {
                final double[] rowdata = toArray(x.get(row));
                for (int col = 0; col < size(); ++col) {
                    rowdata[col] = rowdata[col] - stats[col].mean;
                    rowdata[col] = rowdata[col] / stats[col].stddev;
                }
                System.arraycopy(rowdata, 0, result, row * cols, cols);
            }
            return result;
        }

        /** Distance metric in the normalized space of V. */
        double r(V a, V b, Stats[] stats) {
            double ss = 0.0;
            final double[] adata = toArray(a);
            final double[] bdata = toArray(b);
            for (int col = 0; col < size(); ++col) {
                adata[col] = adata[col] - stats[col].mean;
                adata[col] = adata[col] / stats[col].stddev;
                bdata[col] = bdata[col] - stats[col].mean;
                bdata[col] = bdata[col] / stats[col].stddev;
            }
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
    final Stats[] m_xstats;
    final Stats[] m_ystats;
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

        m_xstats = xadapter.stats(x);
        m_ystats = yadapter.stats(y);

        // yRowMajor is normalized
        double[] yRowMajor = yadapter.rowMajor(m_y, m_ystats);

        double[][] phi = phi(m_x, m_xadapter, m_xstats, m_rbf);
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
        double[] phiVec = phiVec(t, m_x, m_xadapter, m_xstats, m_rbf);
        double[] result = new double[m_yadapter.size()];
        for (int i = 0; i < m_yadapter.size(); ++i) {
            for (int j = 0; j < phiVec.length; ++j) {
                result[i] += phiVec[j] * m_w[j][i];
            }
        }
        // result here is normalized
        for (int i = 0; i < result.length; ++i) {
            result[i] = result[i] * m_ystats[i].stddev;
            result[i] = result[i] + m_ystats[i].mean;
        }
        // now result is denormalized
        return m_yadapter.fromArray(result);
    }

    /** Returns basis function evaluated for each pair in x. */
    static <T> double[][] phi(List<T> x, Adapter<T> xadapter, Stats[] xstats, DoubleUnaryOperator rbf) {
        if (xadapter.size() != xstats.length)
            throw new IllegalArgumentException();
        double[][] phi = new double[x.size()][x.size()];
        for (int i = 0; i < x.size(); ++i) {
            for (int j = 0; j < x.size(); ++j) {
                // r is normalized
                double r = xadapter.r(x.get(i), x.get(j), xstats);
                phi[i][j] = rbf.applyAsDouble(r);
            }
        }
        return phi;
    }

    static <T> double[] phiVec(T p, List<T> x, Adapter<T> xadapter, Stats[] xstats, DoubleUnaryOperator rbf) {
        if (xadapter.size() != xstats.length)
            throw new IllegalArgumentException();
        double[] result = new double[x.size()];
        for (int i = 0; i < x.size(); ++i) {
            // r is normalized
            double r = xadapter.r(p, x.get(i), xstats);
            result[i] = rbf.applyAsDouble(r);
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
