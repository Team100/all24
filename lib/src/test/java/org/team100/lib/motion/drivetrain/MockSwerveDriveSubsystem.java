package org.team100.lib.motion.drivetrain;

import org.team100.lib.geometry.GeometryUtil;
import org.team100.lib.swerve.SwerveSetpoint;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;

public class MockSwerveDriveSubsystem implements SwerveDriveSubsystemInterface {

    public Pose2d pose = GeometryUtil.kPoseZero;
    ChassisSpeeds speeds = new ChassisSpeeds();
    public Twist2d twist = new Twist2d();
    public boolean stopped = false;
    SwerveDriveKinematics kinematics = new SwerveDriveKinematics(
            new Translation2d(0.1, 0.1),
            new Translation2d(0.1, -0.1),
            new Translation2d(-0.1, 0.1),
            new Translation2d(-0.1, -0.1));

    @Override
    public Pose2d getPose() {
        return pose;
    }

    @Override
    public SwerveState getState() {
        return new SwerveState(pose, twist);
    }

    @Override
    public void resetPose(Pose2d robotPose) {
        pose = robotPose;
    }

    @Override
    public void stop() {
        stopped = true;
    }

    @Override
    public SwerveDriveSubsystem get() {
        return null;
    }

    @Override
    public ChassisSpeeds speeds() {
        return speeds;
    }

    @Override
    public void driveInFieldCoords(Twist2d twist) {
        this.twist=twist;
    }

    @Override
    public boolean steerAtRest(Twist2d twist) {
        return true;
    }

    @Override
    public void setChassisSpeeds(ChassisSpeeds speeds) {
        this.speeds = speeds;
    }

    @Override
    public void setRawModuleStates(SwerveModuleState[] states) {
        this.speeds = kinematics.toChassisSpeeds(states);
    }

    @Override
    public boolean[] atSetpoint() {
        return new boolean[] { true, true, true, true };
    }

    @Override
    public boolean[] atGoal() {
        return new boolean[] { true, true, true, true };
    }

    @Override
    public SwerveModuleState[] moduleStates() {
        return null;
    }

    @Override
    public void resetSetpoint(SwerveSetpoint setpoint) {
        // do nothing
    }
}
