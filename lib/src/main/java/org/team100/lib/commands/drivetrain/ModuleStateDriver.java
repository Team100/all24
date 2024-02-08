package org.team100.lib.commands.drivetrain;

import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;

public interface ModuleStateDriver {
    /**
     * @param input control units [-1,1]
     * @return module states
     */
    SwerveModuleState[] apply(Twist2d input);
}
