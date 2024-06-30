package org.team100.lib.motor;

import org.team100.lib.dashboard.Glassy;

public interface BaseMotor100 extends Glassy {

    void stop();

    /**
     * For test cleanup.
     */
    void close();

    @Override
    default String getGlassName() {
        return "Motor100";
    }
}
