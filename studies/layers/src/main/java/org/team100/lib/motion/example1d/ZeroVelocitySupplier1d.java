package org.team100.lib.motion.example1d;

import org.team100.lib.profile.MotionProfile;

/**
 * Always supplies zero, for "safe mode."
 */
public class ZeroVelocitySupplier1d implements ProfileFollower {

    @Override
    public void accept(MotionProfile profile) {
        //
    }

    @Override
    public Double apply(double position_M) {
        return 0.0;
    }
}
