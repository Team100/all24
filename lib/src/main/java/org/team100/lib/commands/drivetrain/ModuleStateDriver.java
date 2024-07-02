package org.team100.lib.commands.drivetrain;

import org.team100.lib.dashboard.Glassy;
import org.team100.lib.hid.DriverControl;
import org.team100.lib.motion.drivetrain.kinodynamics.SwerveModuleState100;


public interface ModuleStateDriver extends Glassy {
    /**
     * @param input control units [-1,1]
     * @return module states
     */
    SwerveModuleState100[] apply(DriverControl.Velocity input);
}
