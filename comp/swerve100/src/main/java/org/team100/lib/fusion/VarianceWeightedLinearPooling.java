package org.team100.lib.fusion;

import org.team100.lib.math.RandomVector;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Num;
import edu.wpi.first.math.Pair;

/**
 * Variance weighted linear pooling is also called "mixing" -- it represents the
 * weighted average of probability functions where the weights are the inverse
 * variances.
 * 
 * This has the advantage of linear mixing: the variance respects the
 * dispersion in means, weighted by their variances (so given two very different
 * variances, the result will be close to the narrow one, but given two equal
 * variances, the result will try to cover both means)
 * 
 * It also the advantage of log-linear mixing: the mean is variance-weighted, so
 * tighter variances "pull" the resulting mean towards themselves.
 * 
 * One thing it should do, but doesn't do, is to respect the covariance implied
 * by the dispersion of means, i.e. shape the resulting variance to minimally
 * contain the mass of the inputs.
 * The weights are as follows:
 * 
 * pa = (1/A)/(1/A + 1/B)
 * pb = (1/B)/(1/A + 1/B)
 * 
 * If you are representing non-Euclidean geometry you'd better be using the
 * right RandomVector class, because this class doesn't know about geometry.
 * 
 * Visualization of this pooling method is available here:
 * 
 * https://colab.research.google.com/drive/1W0YVYi4eXLpfdkSNpOy4otiW2Poliems#scrollTo=ps1ulO5dYUL4
 */
public class VarianceWeightedLinearPooling<States extends Num> extends LinearPooling<States> {
    public static class Config {
        public double kThreshold = 1e-15;
    }

    private final Config m_config = new Config();

    public RandomVector<States> fuse(RandomVector<States> a, RandomVector<States> b) {
        // TODO: turn off these checks somehow for matches, use some sort of backoff
        // strategy
        if (a.getClass() != b.getClass()) {
            throw new IllegalArgumentException("a and b must be same type\n" + a.getClass() + " " + b.getClass());
        }
        Pair<Matrix<States, States>, Matrix<States, States>> weights = weights(a, b);
        Matrix<States, States> pa = weights.getFirst();
        Matrix<States, States> pb = weights.getSecond();
        // System.out.println("pa " + pa);
        // System.out.println("pb " + pb);
        return fuse(a, pa, b, pb);
    }

    /** TODO make a weight type */
    Pair<Matrix<States, States>, Matrix<States, States>> weights(RandomVector<States> a, RandomVector<States> b) {
        Matrix<States, States> aP = a.Kxx.getValue();
        Matrix<States, States> bP = b.Kxx.getValue();
        if (aP.det() < m_config.kThreshold) {
            throw new IllegalArgumentException("aP is singular.\n" + aP.toString());
        }
        if (bP.det() < m_config.kThreshold) {
            throw new IllegalArgumentException("bP is singular.\n" + bP.toString());
        }
        Matrix<States, States> aPI = aP.inv();
        Matrix<States, States> bPI = bP.inv();
        Matrix<States, States> PIsum = aPI.plus(bPI);
        if (PIsum.det() < m_config.kThreshold) {
            throw new IllegalArgumentException("PIsum is singular.\n" + PIsum.toString());
        }
        Matrix<States, States> pIsumI = PIsum.inv();
        Matrix<States, States> pa = aPI.times(pIsumI);
        Matrix<States, States> pb = bPI.times(pIsumI);
        return Pair.of(pa, pb);
    }

}
