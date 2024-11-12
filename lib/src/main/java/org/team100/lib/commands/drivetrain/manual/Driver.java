package org.team100.lib.commands.drivetrain.manual;

import org.team100.lib.hid.DriverControl;
import org.team100.lib.motion.drivetrain.SwerveModel;

import edu.wpi.first.math.geometry.Pose2d;

public interface Driver {
    void apply(SwerveModel s, DriverControl.Velocity t);

    void reset(Pose2d p);
}