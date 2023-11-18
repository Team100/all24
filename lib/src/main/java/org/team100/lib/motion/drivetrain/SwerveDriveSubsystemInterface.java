package org.team100.lib.motion.drivetrain;

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

    void setChassisSpeeds(ChassisSpeeds speeds);

    void setRawModuleStates(SwerveModuleState[] states);

    void stop();

    //
    //
    ///////////////////////////////

    Pose2d getPose();

    ChassisSpeeds speeds();

    /** Because Subsystem is now concrete, it needs an accessor. */
    SwerveDriveSubsystem get();
}
