package org.team100.lib.config;

import org.team100.lib.copies.SwerveDrivePoseEstimator100;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;

public class NotePoseDetector {
    private final SwerveDrivePoseEstimator100 m_swervePose;
    private Translation2d m_robotRelativeNoteTranslation = new Translation2d();

    /**
     * @param swervePose The swerve pose estimator
     */
    public NotePoseDetector(
            SwerveDrivePoseEstimator100 swervePose) {
        m_swervePose = swervePose;
    }

    /**
     * Updates the pose detector with the correct robot relative pose
     */
    public void update(Translation2d robotRelativeNoteTranslation) {
        m_robotRelativeNoteTranslation = robotRelativeNoteTranslation;
    }

    /**
     * @return A robot relative translational x value of an object in a camera in
     *         meters
     */
    public double getX() {
        return m_robotRelativeNoteTranslation.getX();
    }

    /**
     * @return A field relative translational x value of an object in a camera in
     *         meters
     */
    public Double fieldRelativeX() {
        switch (Identity.instance) {
            case BETA_BOT:
            case COMP_BOT:
                return fieldRelativePose2d().getX();
            case BLANK:
                return 0.0;
            default:
                return 0.0;
        }
    }

    /**
     * @return A robot relative angle in radians to the note
     */
    public Rotation2d robotRelativeAngleToNote() {
        return m_robotRelativeNoteTranslation.getAngle();
    }

    /**
     * @return A field relative angle in radians to the note
     */
    public Rotation2d fieldRelativeAngleToNote() {
        switch (Identity.instance) {
            case BETA_BOT:
            case COMP_BOT:
                return fieldRelativePose2d().getRotation();
            case BLANK:
                return FieldRelativeTranslation2d().minus(m_swervePose.getEstimatedPosition().getTranslation())
                        .getAngle();
            default:
                return FieldRelativeTranslation2d().minus(m_swervePose.getEstimatedPosition().getTranslation())
                        .getAngle();
        }
    }

    /**
     * @return A robot relative translational y value of an object in a camera in
     *         meters
     */
    public Double getY() {
        return m_robotRelativeNoteTranslation.getY();
    }

    /**
     * @return A field relative translational y value of an object in a camera in
     *         meters
     */
    public Double fieldRelativeY() {
        switch (Identity.instance) {
            case BETA_BOT:
            case COMP_BOT:
                return fieldRelativePose2d().getY();
            case BLANK:
                return 0.0;
            default:
                return 0.0;
        }
    }

    /**
     * @return A robot relative translational value of an object in a camera in
     *         meters, rot value is bearing
     */
    public Translation2d RobotRelativeTranslation2d() {
        return m_robotRelativeNoteTranslation;
    }

    /**
     * @return A field relative translational value of an object in a camera in
     *         meters
     */
    public Translation2d FieldRelativeTranslation2d() {
        return new Translation2d(fieldRelativeX(), fieldRelativeY());
    }

    /**
     * @return A robot relative Transform2d value of an object in a camera in meters
     *         for translation and radians for rotation
     */
    public Transform2d robotRelativeTransform2d() {
        return new Transform2d(m_robotRelativeNoteTranslation, robotRelativeAngleToNote());
    }

    /**
     * @return A field relative Pose2d value of an object in a camera in meters for
     *         translation and radians for rotation
     */
    public Pose2d fieldRelativePose2d() {
        switch (Identity.instance) {
            case BETA_BOT:
            case COMP_BOT:
                return m_swervePose.getEstimatedPosition().transformBy(robotRelativeTransform2d());
            case BLANK:
                return new Pose2d(fieldRelativeX(), fieldRelativeY(), fieldRelativeAngleToNote());
            default:
                return new Pose2d(fieldRelativeX(), fieldRelativeY(), fieldRelativeAngleToNote());
        }
    }
}