package org.team100.lib.profile;

import org.team100.lib.state.State100;

public interface Profile100 {

    public static record ResultWithETA(State100 state, double etaS) {
    }

    /** Note order here, initial first, goal second. */
    State100 calculate(double dt, State100 initial, State100 goal);

    ResultWithETA calculateWithETA(double dt, State100 initial, State100 goal);

}