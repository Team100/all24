package org.team100.sim;

import org.dyn4j.collision.Filter;

/** Objects being carried don't interact with anything. */
public class CarryFilter implements Filter {

    @Override
    public boolean isAllowed(Filter filter) {
        return false;
    }

}
