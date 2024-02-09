package org.team100.lib.config;

import org.team100.lib.localization.NotePosition24ArrayListener;
import org.team100.lib.motion.drivetrain.SwerveDriveSubsystem;
import org.team100.lib.util.CameraAngles;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;

public class NoteDetector {
    private final CameraAngles m_cameraAngles;
    private final SwerveDriveSubsystem m_swerve;
    private final NotePosition24ArrayListener m_notePosition24ArrayListener;

    /**
     * @param cameraAngles                The camera class with the parameters of
     *                                    the camera
     * @param notePosition24ArrayListener Class which gets the x and a values of the
     *                                    object detected from the PI
     * @param swerve                      The swerve drivetrain
     */
    public NoteDetector(
            CameraAngles cameraAngles,
            NotePosition24ArrayListener notePosition24ArrayListener,
            SwerveDriveSubsystem swerve) {
        m_cameraAngles = cameraAngles;
        m_notePosition24ArrayListener = notePosition24ArrayListener;
        m_swerve = swerve;
    }

    /**
     * @return A robot relative translational x value of an object in a camera in
     *         meters
     */
    public double getX() {
        return m_cameraAngles.getX(m_notePosition24ArrayListener.getY().get());
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
        return new Rotation2d(m_cameraAngles.getAngle(m_notePosition24ArrayListener.getX().get(),
                m_notePosition24ArrayListener.getY().get()));
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
                return FieldRelativeTranslation2d().minus(m_swerve.getPose().getTranslation()).getAngle();
            default:
                return FieldRelativeTranslation2d().minus(m_swerve.getPose().getTranslation()).getAngle();
            }
    }

    /**
     * @return A robot relative translational y value of an object in a camera in
     *         meters
     */
    public Double getY() {
        return m_cameraAngles.getY(m_notePosition24ArrayListener.getX().get(),
                m_notePosition24ArrayListener.getY().get());
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
        return new Translation2d(getX(), getY());
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
        return new Transform2d(getX(), getY(), robotRelativeAngleToNote());
    }

    /**
     * @return A field relative Pose2d value of an object in a camera in meters for
     *         translation and radians for rotation
     */
    public Pose2d fieldRelativePose2d() {
        switch (Identity.instance) {
            case BETA_BOT:
            case COMP_BOT:
                return m_swerve.getPose().transformBy(robotRelativeTransform2d());
            case BLANK:
                return new Pose2d(fieldRelativeX(), fieldRelativeY(), fieldRelativeAngleToNote());
            default:
                return new Pose2d(fieldRelativeX(), fieldRelativeY(), fieldRelativeAngleToNote());
        }
    }
}