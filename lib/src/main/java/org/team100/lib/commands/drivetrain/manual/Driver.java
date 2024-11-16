package org.team100.lib.commands.drivetrain.manual;

import org.team100.lib.hid.DriverControl;
import org.team100.lib.motion.drivetrain.SwerveModel;

public interface Driver {
    void apply(SwerveModel s, DriverControl.Velocity t);

    void reset(SwerveModel s);
}