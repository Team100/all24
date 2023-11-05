package org.team100.lib.fusion;

import org.team100.lib.math.RandomVector;

import edu.wpi.first.math.Num;

/**
 * Given two estimates for the same thing, return an aggregate. There are lots
 * of ways to do it.
 */
public interface Pooling<States extends Num> {
    RandomVector<States> fuse(RandomVector<States> a, RandomVector<States> b);
}
