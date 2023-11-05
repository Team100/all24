package org.team100.lib.math;

import edu.wpi.first.math.Num;
import edu.wpi.first.math.numbers.N2;

/**
 * White noise, w in the state equation:
 * 
 * xdot = f(x, u, t) + w(t)
 * 
 * also sometimes xi or ξ or η.
 * 
 * White noise is a process that produces uncorrelated
 * samples drawn from a gaussian distribution with zero mean and the specified
 * covariance. It's not a random variable (which represents a belief about a
 * hidden state), it is the state itself. About the only thing you can do with
 * noise is integrate it (as described by Norbert Wiener, so the "Wiener
 * process") which *does* produce a normal random variable with mean of zero and
 * variance of t times the underlying distribution, representing the resulting
 * "random walk". For comparison integration of a random variable produces t
 * *squared* times the underlying distribution: integrating noise produces less
 * variance than integrating a belief.
 */
public class WhiteNoiseVector<States extends Num> {
    public final Variance<States> P;

    public WhiteNoiseVector(Variance<States> P) {
        this.P = P;
    }

    public static WhiteNoiseVector<N2> noise2(double w1, double w2) {
        Variance<N2> kxx = Variance.from2StdDev(w1, w2);
        return new WhiteNoiseVector<>(kxx);
    }
}
