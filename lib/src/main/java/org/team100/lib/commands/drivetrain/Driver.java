package org.team100.lib.commands.drivetrain;

import org.team100.lib.motion.drivetrain.SwerveState;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Twist2d;

public interface Driver {
    void apply(SwerveState s, Twist2d t, double dt);

    void reset(Pose2d p);
}