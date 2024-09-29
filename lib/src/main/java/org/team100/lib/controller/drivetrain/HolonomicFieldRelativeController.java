package org.team100.lib.controller.drivetrain;

import org.team100.lib.dashboard.Glassy;
import org.team100.lib.motion.drivetrain.SwerveState;
import org.team100.lib.motion.drivetrain.kinodynamics.FieldRelativeVelocity;

import edu.wpi.first.math.geometry.Pose2d;

public interface HolonomicFieldRelativeController extends Glassy {

    /**
     * @param currentPose  robot's current pose in field coordinates
     * @param desiredState reference state
     * @return field-relative , meters and radians per second
     */
    FieldRelativeVelocity calculate(Pose2d currentPose, SwerveState desiredState);

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
