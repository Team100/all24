package org.team100.lib.profile;

import org.team100.lib.state.State100;

/** Always returns the initial state. */
public class NullProfile implements Profile100 {

    @Override
    public State100 calculate(double dt, State100 initial, State100 goal) {
        return initial;
    }
    
}
