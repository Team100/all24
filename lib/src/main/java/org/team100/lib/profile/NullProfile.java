package org.team100.lib.profile;

import org.team100.lib.state.Control100;
import org.team100.lib.state.Model100;

/** Always returns the initial state. */
public class NullProfile implements Profile100 {

    @Override
    public Control100 calculate(double dt, Model100 initial, Model100 goal) {
        return initial.control();
    }

    @Override
    public ResultWithETA calculateWithETA(double dt, Model100 initial, Model100 goal) {
        return new ResultWithETA(initial.control(), 0);
    }

}
