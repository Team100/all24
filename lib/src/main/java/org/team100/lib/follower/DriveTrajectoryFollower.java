package org.team100.lib.follower;

import org.team100.lib.dashboard.Glassy;
import org.team100.lib.trajectory.TrajectoryTimeIterator;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

/**
 * Interface for multiple types of drivetrain trajectory followers.
 */
public interface DriveTrajectoryFollower extends Glassy {

    void setTrajectory(TrajectoryTimeIterator trajectory);

    /**
     * Makes no attempt to enforce feasibility.
     * 
     * @param timestamp        in seconds, use Timer.getFPGATimestamp()
     * @param measurement      measured pose
     * @param current_velocity measured robot-relative speed, this should be
     *                         obtained from drivetrain.speeds().
     * @return velocity control input
     */
    ChassisSpeeds update(double timestamp, Pose2d measurement, ChassisSpeeds current_velocity);

    boolean isDone();
}
