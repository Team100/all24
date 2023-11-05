package org.team100.lib.fusion;

import org.team100.lib.math.RandomVector;
import org.team100.lib.math.Variance;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Num;
import edu.wpi.first.math.numbers.N1;

/**
 * A geometric mixture of probability functions;
 * 
 * P = product(P_i^w_i)
 * 
 * For gaussian random variables, for mean a,b,c and covariance A, B, C:
 * 
 * c = C*(wa/A + (1-w)b/B)
 * C = 1/(w/A + (1-w)/B)
 * 
 * How does this pooling behave?
 * 
 * The aggregate mean is weighted by the covariances, which seems right:
 * more-precise estimates have more influence.
 * 
 * The aggregate covariance doesn't depend on the means, which seems wrong.
 * 
 * This pooling method cannot handle zero variances.
 * 
 * Bayesian inference and inverse variance weighting are the same idea, without
 * the 'w' factor.
 */
public abstract class LogLinearPooling<States extends Num> implements Pooling<States> {

    /**
     * Weights should add to one.
     * 
     * @param pa a weight
     * @param pb b weight
     */
    RandomVector<States> fuse(RandomVector<States> a, double pa, RandomVector<States> b, double pb) {
        Matrix<States, N1> ax = a.x;
        Matrix<States, N1> bx = b.x;
        Variance<States> aP = a.Kxx;
        Variance<States> bP = b.Kxx;

        Matrix<States, States> paaPI = aP.getValue().inv().times(pa);
        Matrix<States, States> pbbPI = bP.getValue().inv().times(pb);
        Matrix<States, States> cP = paaPI.plus(pbbPI).inv();

        Matrix<States, N1> cax = aP.getValue().inv().times(ax).times(pa);
        Matrix<States, N1> cbx = bP.getValue().inv().times(bx).times(pb);
        
        Matrix<States, N1> cx = cP.times(cax.plus(cbx));

        return a.make(cx, new Variance<>(cP));
    }
}
