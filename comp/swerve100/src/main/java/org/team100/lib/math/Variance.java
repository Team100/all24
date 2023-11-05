package org.team100.lib.math;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.Num;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;

/**
 * Variance, aka Covariance Matrix, represents the n-dimensional variance of a
 * random variable.
 * 
 * https://en.wikipedia.org/wiki/Covariance_matrix
 */
public class Variance<Dim extends Num> {
    /** Very high variance represents unknown variable. */
    private static final double kDontKnow = 1e9;

    private final Matrix<Dim, Dim> value;

    public Variance(Matrix<Dim, Dim> value) {
        this.value = value;
    }

    /** Set row i to very high variance. */
    public void dontknow(int i) {
        value.set(i, i, kDontKnow);
    }

    /** Make a diagonal covarince matrix from a vector of standard deviations. */
    public static <D extends Num> Variance<D> fromStdDev(Nat<D> dim, Matrix<D, N1> stdDev) {
        Matrix<D, D> kxx = new Matrix<>(dim, dim);
        for (int i = 0; i < dim.getNum(); ++i) {
            kxx.set(i, i, Math.pow(stdDev.get(i, 0), 2));
        }
        return new Variance<>(kxx);
    }

    /** Specialization for the common case of two std dev's */
    public static Variance<N2> from2StdDev(double sigma1, double sigma2) {
        Matrix<N2, N2> kxx = new Matrix<>(Nat.N2(), Nat.N2());
        kxx.set(0, 0, Math.pow(sigma1, 2));
        kxx.set(1, 1, Math.pow(sigma2, 2));
        return new Variance<>(kxx);
    }

    public static <D extends Num> Variance<D> zero(Nat<D> dim) {
        Matrix<D, D> kxx = new Matrix<>(dim, dim);
        return new Variance<>(kxx);
    }

    /** Specialization for the common case of 2d zero */
    public static Variance<N2> zero2() {
        return zero(Nat.N2());
    }

    public Variance<Dim> copy() {
        return new Variance<>(value.copy());
    }

    public Variance<Dim> plus(Variance<Dim> other) {
        return new Variance<>(value.plus(other.value));
    }

    public Variance<Dim> minus(Variance<Dim> other) {
        return new Variance<>(value.minus(other.value));
    }

    public Variance<Dim> times(double a) {
        return new Variance<>(value.times(a));
    }

    public Variance<Dim> times(Variance<Dim> other) {
        return new Variance<>(value.times(other.value));
    }

    // for testing
    public double[] getData() {
        return value.getData();
    }

    public double get(int i, int j) {
        return value.get(i,j);
    }

    public Matrix<Dim, Dim> getValue() {
        return value;
    }

    @Override
    public String toString() {
        return "Variance [value=" + value + "]";
    }
}
