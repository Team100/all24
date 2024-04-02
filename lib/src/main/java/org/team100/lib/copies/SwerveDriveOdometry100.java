package org.team100.lib.copies;

import org.team100.lib.util.DriveUtil;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.SwerveDriveWheelPositions;
import edu.wpi.first.math.kinematics.SwerveModulePosition;

/**
 * Collapses WPI SwerveDriveOdometry and Odometry.
 */
public class SwerveDriveOdometry100 {
    private final int m_numModules;

    private final SwerveDriveKinematics100 m_kinematics;

    /**
     * "current" pose, maintained in update() and resetPosition().
     */
    private Pose2d m_poseMeters;

    private Pose2d m_previousPoseMeters;
    private double m_previousTimeSeconds;

    /**
     * maintained in resetPosition().
     */
    private Rotation2d m_gyroOffset;

    /**
     * maintained in update() as gyro angle plus offset.
     */
    private Rotation2d m_previousAngle;

    /**
     * maintained in update() and resetPosition()
     */
    private SwerveDriveWheelPositions m_previousWheelPositions;

    /**
     * @param kinematics      The swerve drive kinematics for your drivetrain.
     * @param gyroAngle       The angle reported by the gyroscope.
     * @param modulePositions The wheel positions reported by each module.
     * @param initialPose     The starting position of the robot on the field.
     */
    public SwerveDriveOdometry100(
            SwerveDriveKinematics100 kinematics,
            Rotation2d gyroAngle,
            SwerveModulePosition[] modulePositions,
            Pose2d initialPose) {
        m_numModules = modulePositions.length;
        m_kinematics = kinematics;
        m_poseMeters = initialPose;
        m_gyroOffset = m_poseMeters.getRotation().minus(gyroAngle);
        m_previousAngle = m_poseMeters.getRotation();
        m_previousWheelPositions = new SwerveDriveWheelPositions(modulePositions).copy();
    }

    /**
     * Resets the robot's position on the field.
     *
     * The gyroscope angle does not need to be reset here on the user's robot code.
     * The library automatically takes care of offsetting the gyro angle.
     *
     * Similarly, module positions do not need to be reset in user code.
     *
     * @param gyroAngle       The angle reported by the gyroscope.
     * @param modulePositions The wheel positions reported by each module.,
     * @param pose            The position on the field that your robot is at.
     */
    public void resetPosition(
            Rotation2d gyroAngle,
            SwerveDriveWheelPositions modulePositions,
            Pose2d pose) {
        checkLength(modulePositions);
        m_poseMeters = pose;
        m_previousAngle = m_poseMeters.getRotation();
        m_gyroOffset = m_poseMeters.getRotation().minus(gyroAngle);
        m_previousWheelPositions = modulePositions.copy();
    }

    public Rotation2d getGyroOffset() {
        return m_gyroOffset;
    }

    /**
     * Used only by the pose estimator.
     */
    Pose2d getPoseMeters() {
        return m_poseMeters;
    }

    /**
     * Updates the robot's position on the field using forward kinematics.
     *
     * @param currentTimeSeconds
     * @param gyroAngle          from gyro.
     * @param modulePositions    The current position of all swerve modules.
     * @return The new pose of the robot.
     */
    public Pose2d update(
            double currentTimeSeconds,
            Rotation2d gyroAngle,
            SwerveDriveWheelPositions modulePositions) {
        checkLength(modulePositions);

        Rotation2d angle = gyroAngle.plus(m_gyroOffset);

        Twist2d twist = m_kinematics.toTwist2d(
                DriveUtil.modulePositions(m_previousWheelPositions, modulePositions));
        twist.dtheta = angle.minus(m_previousAngle).getRadians();

        Pose2d newPose = m_poseMeters.exp(twist);

        m_previousWheelPositions = modulePositions.copy();
        m_previousAngle = angle;
        m_poseMeters = new Pose2d(newPose.getTranslation(), angle);

        return m_poseMeters;
    }

    ///////////////////////////////////////

    private void checkLength(SwerveDriveWheelPositions modulePositions) {
        int ct = modulePositions.positions.length;
        if (ct != m_numModules) {
            throw new IllegalArgumentException("Wrong module count: " + ct);
        }
    }
}
