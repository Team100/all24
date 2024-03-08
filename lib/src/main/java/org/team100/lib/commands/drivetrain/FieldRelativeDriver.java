package org.team100.lib.commands.drivetrain;

import org.team100.lib.dashboard.Glassy;
import org.team100.lib.motion.drivetrain.SwerveState;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Twist2d;

public interface FieldRelativeDriver extends Glassy {
    /**
     * @param state from the drivetrain
     * @param input control units [-1,1]
     * @return feasible field-relative velocity in m/s and rad/s
     */
    Twist2d apply(SwerveState state, Twist2d input);

    void reset(Pose2d p);

}
