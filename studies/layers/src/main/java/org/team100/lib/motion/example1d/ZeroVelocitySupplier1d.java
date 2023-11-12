package org.team100.lib.motion.example1d;

import org.team100.lib.motion.example1d.framework.Workstate;

/**
 * Always supplies zero, for "safe mode."
 */
public class ZeroVelocitySupplier1d implements ProfileFollower {


    @Override
    public Workstate<Double> apply(Workstate<Double> position_M) {
        return new CrankWorkstate(0.0);
    }
}
