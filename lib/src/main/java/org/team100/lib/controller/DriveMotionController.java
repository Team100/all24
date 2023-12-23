package org.team100.lib.controller;

import org.team100.lib.trajectory.TrajectoryTimeIterator;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

/** 
 * Interface for multiple types of drivetrain trajectory followers.
 */
public interface DriveMotionController {

    void setTrajectory(TrajectoryTimeIterator trajectory);

    /** 
     * @param timestamp in seconds, use Timer.getFPGATimestamp()
     * @param measurement measured pose
     * @param current_velocity measured speed
     * @return velocity control input
     */
    ChassisSpeeds update(double timestamp, Pose2d measurement, Twist2d current_velocity);

    boolean isDone();
}
