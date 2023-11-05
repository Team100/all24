package org.team100.lib.motion.drivetrain;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

/** Minimal interface makes testing easier. */
public interface SwerveDriveSubsystemInterface {

    Pose2d getPose();

    void stop();

    void setDesiredState(SwerveState desiredState);

    ChassisSpeeds speeds();

    void truncate();

    /** Because Subsystem is now concrete, it needs an accessor. */
    SwerveDriveSubsystemInterface get();
}
