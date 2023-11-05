package org.team100.lib.fusion;

import org.team100.lib.math.RandomVector;

import edu.wpi.first.math.Num;

/**
 * Democratic pooling assigns equal weight to each input.
 */
public class DemocraticLinearPooling<States extends Num> extends LinearPooling<States> {
    public RandomVector<States> fuse(RandomVector<States> a, RandomVector<States> b) {
        return fuse(a, 0.5, b, 0.5);
    }
}
