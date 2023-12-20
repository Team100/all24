package org.team100.lib.motion.drivetrain;

import org.team100.lib.swerve.SwerveSetpoint;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;

/** Minimal interface makes testing easier. */
public interface SwerveDriveSubsystemInterface {

    ///////////////////////////////////
    //
    // DRIVE METHODS
    //
    // there are four mutually exclusive drive methods
    // we depend on CommandScheduler to enforce the mutex.

    void driveInFieldCoords(Twist2d twist);
    
    /** @return true if aligned */
    boolean steerAtRest(Twist2d twist);

    void setChassisSpeeds(ChassisSpeeds speeds);

    void setRawModuleStates(SwerveModuleState[] states);

    void stop();

    //
    //
    ///////////////////////////////

    Pose2d getPose();

    SwerveState getState();

    void resetPose(Pose2d robotPose);

    /** The controllers are on the profiles. */
    boolean[] atSetpoint();

    /** The profiles setpoints are at their goals. */
    boolean[] atGoal();

    ChassisSpeeds speeds();

    SwerveModuleState[] moduleStates();

    void resetSetpoint(SwerveSetpoint setpoint);

    /** Because Subsystem is now concrete, it needs an accessor. */
    SwerveDriveSubsystem get();
}
