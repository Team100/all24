package org.team100.lib.math;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Num;
import edu.wpi.first.math.numbers.N1;

/**
 * A vector-valued gaussian random variable, which represents a belief about a
 * hidden state.
 * 
 * https://en.wikipedia.org/wiki/Multivariate_random_variable
 * 
 * This class may be used for measurements, or for state estimates.
 */
public class RandomVector<States extends Num> {
    /**
     * Expected value
     * https://en.wikipedia.org/wiki/Expected_value
     */
    public final Matrix<States, N1> x;
    /**
     * Variance
     * https://en.wikipedia.org/wiki/Covariance_matrix
     */
    public final Variance<States> Kxx;

    public RandomVector(Matrix<States, N1> x, Variance<States> Kxx) {
        this.x = x;
        this.Kxx = Kxx;
    }

    /** Instantiation that preserves type, i.e. it can be overridden. */
    public RandomVector<States> make(Matrix<States, N1> x, Variance<States> Kxx) {
        return new RandomVector<>(x, Kxx);
    }

    public RandomVector<States> copy() {
        return make(x.copy(), Kxx.copy());
    }

    /**
     * Mean and covariance are simply added, which corresponds to assuming the
     * variables are independent. This is Euclidean.
     */
    public RandomVector<States> plus(RandomVector<States> b) {
        return make(x.plus(b.x), Kxx.plus(b.Kxx));
    }

    /**
     * Mean is subtracted, covariance is *added* which corresponds to assuming the
     * variables are independent. This is Euclidean.
     */
    public RandomVector<States> minus(RandomVector<States> b) {
        return make(x.minus(b.x), Kxx.plus(b.Kxx));
    }

    /** Euclidean version */
    public Matrix<States, N1> xplus(Matrix<States, N1> otherx) {
        return this.x.plus(otherx);
    }

    /**
     * Euclidean version, makes the expressions a little cleaner to have both plus
     * and minus.
     */
    public Matrix<States, N1> xminus(Matrix<States, N1> otherx) {
        return this.x.minus(otherx);
    }

    /**
     * Weighted average with another variable.
     * 
     * @param weight the weight of the other variable, assuming the weight of this
     *               variable is 1-weight.
     * @return this + weight * (other - this)
     */
    public RandomVector<States> combine(Matrix<States, States> weight, RandomVector<States> other) {
        Matrix<States, N1> xx = xplus(weight.times(other.xminus(this.x)));
        // so this is [1 0 0 1e9]
        // other is [1e9 0 0 1]
        // so the diff should be [1e9 0 0 -1e9]
        Variance<States> diff = other.Kxx.minus(this.Kxx);
      // System.out.println("diff " + diff);
        // thisvalue should be [1 0 0 1e9]
        Matrix<States, States> thisValue = this.Kxx.getValue();
      // System.out.println("thisValue " + thisValue);
      //  System.out.println("weight " + weight);
        // weight of other is [1e-9 0 0 1]
        // multiplying 1e9 times 1e-9 gives 1 not zero. sigh.
        // this is the problem

        Matrix<States, States> PP = thisValue.plus(
            weight.times(
                diff.getValue()));

       // System.out.println("PP " + PP);
        return make(xx, new Variance<>(PP));
    }

    /**
     * Scalar multiplication. Remember that the scalar is *squared* before applying
     * to the covariance.
     */
    public RandomVector<States> times(double d) {
        return make(x.times(d), Kxx.times(d * d));
    }

    @Override
    public String toString() {
        return "RandomVector [x=" + x + ", P=" + Kxx + "]";
    }

}
