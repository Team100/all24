package org.team100.lib.motion.example1d;

import java.util.function.Function;

import org.team100.lib.motion.example1d.framework.Workstate;

/**
 * A profile follower accepts profiles and then begins supplying velocity
 * setpoints based on an internal timer and supplied state. This is intended to
 * be instantiated and used once per command invocation.
 * 
 * // TODO i think this shouldn't get position.
 */
public interface ProfileFollower extends Function<Workstate<Double>,Workstate<Double>> {

    /**
     * @param position_M current state, meters
     * @return control output, u, meters per second
     */
    @Override
    Workstate<Double> apply(Workstate<Double> position_M);
}
