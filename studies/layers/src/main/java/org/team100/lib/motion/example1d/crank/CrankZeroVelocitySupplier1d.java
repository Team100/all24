package org.team100.lib.motion.example1d.crank;

import org.team100.lib.profile.MotionProfile;

/**
 * Always supplies zero, for "safe mode."
 */
public class CrankZeroVelocitySupplier1d implements CrankProfileFollower {

    @Override
    public CrankProfileFollower withProfile(MotionProfile profile) {
        // ignore the profile
        return this;
    }

    @Override
    public CrankWorkstate get() {
        return new CrankWorkstate(0.0);
    }
}
