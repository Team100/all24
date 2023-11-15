package org.team100.lib.motion.example1d.crank;

import org.team100.lib.profile.MotionProfile;

/**
 * A profile follower accepts profiles and then begins supplying velocity
 * setpoints based on an internal timer and supplied state. This is intended to
 * be instantiated and used once per command invocation.
 * 
 * this is a weird follower that gets positions and produces positions? eh?
 * 
 * // TODO i think this shouldn't get position.
 */
public interface CrankProfileFollower {

    CrankProfileFollower withProfile(MotionProfile profile);

    /**
     * @param position_M current state, meters
     * @return control output, u, meters per second
     */
    CrankWorkstate apply(CrankWorkstate position_M);

    /**
     * use the supplier as input
     * 
     * @return control output, u, meters per second */
    CrankWorkstate calculate();
}
