package org.team100.lib.commands.drivetrain;

import org.team100.lib.hid.DriverControl;
import org.team100.lib.motion.drivetrain.SwerveState;

import edu.wpi.first.math.geometry.Pose2d;

public interface Driver {
    void apply(SwerveState s, DriverControl.Velocity t, double dt);

    void reset(Pose2d p);
}