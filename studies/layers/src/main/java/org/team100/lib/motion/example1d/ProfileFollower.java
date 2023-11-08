package org.team100.lib.motion.example1d;

import java.util.function.Consumer;
import java.util.function.DoubleFunction;

import org.team100.lib.profile.MotionProfile;

/**
 * A profile follower accepts profiles and then begins supplying velocity
 * setpoints based on an internal timer and supplied state. This is intended to
 * be instantiated and used once per command invocation.
 */
public interface ProfileFollower extends Consumer<MotionProfile>, DoubleFunction<Double> {

    /**
     * Accept a motion profile and start the timer.
     */
    @Override
    void accept(MotionProfile profile);

    /**
     * @param position_M current state, meters
     * @return control output, u, meters per second
     */
    @Override
    Double apply(double position_M);
}
