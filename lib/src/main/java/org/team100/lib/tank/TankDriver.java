package org.team100.lib.tank;

import org.team100.lib.hid.DriverControl;

public interface TankDriver {
    void apply(DriverControl.Velocity t, double dt);
}
