package org.team100.lib.commands.drivetrain.manual;

import org.team100.lib.dashboard.Glassy;
import org.team100.lib.hid.DriverControl;
import org.team100.lib.motion.drivetrain.SwerveModel;
import org.team100.lib.motion.drivetrain.kinodynamics.FieldRelativeVelocity;

import edu.wpi.first.math.geometry.Pose2d;

public interface FieldRelativeDriver extends Glassy {
    /**
     * @param state from the drivetrain
     * @param input control units [-1,1]
     * @return feasible field-relative velocity in m/s and rad/s
     */
    FieldRelativeVelocity apply(SwerveModel state, DriverControl.Velocity input);

    void reset(Pose2d p);

}
