package org.team100.lib.profile;

import org.team100.lib.controller.State100;

public interface Profile100 {

    /** Note order here, initial first, goal second. */
    State100 calculate(double dt, State100 initial, State100 goal);

}