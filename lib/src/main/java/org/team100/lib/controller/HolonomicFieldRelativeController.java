package org.team100.lib.controller;

import org.team100.lib.dashboard.Glassy;
import org.team100.lib.motion.drivetrain.SwerveState;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Twist2d;

public interface HolonomicFieldRelativeController extends Glassy {

    /**
     * @param currentPose  robot's current pose in field coordinates
     * @param desiredState reference state
     * @return field-relative twist, meters and radians per second
     */
    Twist2d calculate(Pose2d currentPose, SwerveState desiredState);

    /**
     * This uses the tolerances in the controllers.
     * 
     * @return True if the pose error is within tolerance of the reference.
     */
    boolean atReference();

    /**
     * Reset controller state, e.g. velocity error.
     */
    void reset();

}
