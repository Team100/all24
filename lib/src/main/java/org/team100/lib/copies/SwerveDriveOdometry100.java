package org.team100.lib.copies;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
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

    // /**
    // * Constructs a SwerveDriveOdometry object with the default pose at the
    // origin.
    // *
    // * @param kinematics The swerve drive kinematics for your drivetrain.
    // * @param gyroAngle The angle reported by the gyroscope.
    // * @param modulePositions The wheel positions reported by each module.
    // */
    // public SwerveDriveOdometry100(
    // SwerveDriveKinematics100 kinematics,
    // Rotation2d gyroAngle,
    // SwerveModulePosition[] modulePositions) {
    // this(kinematics, gyroAngle, modulePositions, new Pose2d());
    // }

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
            Rotation2d gyroAngle, SwerveModulePosition[] modulePositions, Pose2d pose) {
        resetPosition(gyroAngle, new SwerveDriveWheelPositions(modulePositions), pose);
    }

    public void resetPosition(
            Rotation2d gyroAngle,
            SwerveDriveWheelPositions modulePositions,
            Pose2d pose) {
        if (modulePositions.positions.length != m_numModules) {
            throw new IllegalArgumentException(
                    "Number of modules is not consistent with number of wheel locations provided in "
                            + "constructor");
        }

        m_poseMeters = pose;
        m_previousAngle = m_poseMeters.getRotation();
        m_gyroOffset = m_poseMeters.getRotation().minus(gyroAngle);
        m_previousWheelPositions = modulePositions.copy();
    }

    public Rotation2d getGyroOffset() {
        return m_gyroOffset;
    }

    /**
     * Returns the position of the robot on the field.
     * 
     * TODO: remove this since it depends on calling update() first.
     *
     * @return The pose of the robot (x and y are in meters).
     */
    public Pose2d getPoseMeters() {
        return m_poseMeters;
    }

    /**
     * Updates the robot's position on the field using forward kinematics and
     * integration of the pose
     * over time. This method automatically calculates the current time to calculate
     * period
     * (difference between two timestamps). The period is used to calculate the
     * change in distance
     * from a velocity. This also takes in an angle parameter which is used instead
     * of the angular
     * rate that is calculated from forward kinematics.
     *
     * @param gyroAngle       The angle reported by the gyroscope.
     * @param modulePositions The current position of all swerve modules. Please
     *                        provide the positions
     *                        in the same order in which you instantiated your
     *                        SwerveDriveKinematics.
     * @return The new pose of the robot.
     */
    public Pose2d update(Rotation2d gyroAngle, SwerveModulePosition[] modulePositions) {
        return update(gyroAngle, new SwerveDriveWheelPositions(modulePositions));
    }

    public Pose2d update(double currentTimeSeconds, Rotation2d gyroAngle, SwerveModulePosition[] modulePositions) {
        return update(currentTimeSeconds, gyroAngle, new SwerveDriveWheelPositions(modulePositions));
    }

    public Pose2d update(Rotation2d gyroAngle, SwerveDriveWheelPositions modulePositions) {
        if (modulePositions.positions.length != m_numModules) {
            throw new IllegalArgumentException(
                    "Number of modules is not consistent with number of wheel locations provided in "
                            + "constructor");
        }

        var angle = gyroAngle.plus(m_gyroOffset);

        var twist = m_kinematics.toTwist2d(m_previousWheelPositions, modulePositions);
        twist.dtheta = angle.minus(m_previousAngle).getRadians();

        var newPose = m_poseMeters.exp(twist);

        m_previousWheelPositions = modulePositions.copy();
        m_previousAngle = angle;
        m_poseMeters = new Pose2d(newPose.getTranslation(), angle);

        return m_poseMeters;

    }

    public Pose2d update(
            double currentTimeSeconds,
            Rotation2d gyroAngle,
            SwerveDriveWheelPositions modulePositions) {
        if (modulePositions.positions.length != m_numModules) {
            throw new IllegalArgumentException(
                    "Number of modules is not consistent with number of wheel locations provided in "
                            + "constructor");
        }

        var angle = gyroAngle.plus(m_gyroOffset);

        var twist = m_kinematics.toTwist2d(m_previousWheelPositions, modulePositions);
        twist.dtheta = angle.minus(m_previousAngle).getRadians();

        var newPose = m_poseMeters.exp(twist);

        m_previousWheelPositions = modulePositions.copy();
        m_previousAngle = angle;
        m_poseMeters = new Pose2d(newPose.getTranslation(), angle);

        return m_poseMeters;

    }
}
