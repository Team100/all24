package org.team100.lib.config;

import org.team100.lib.localization.NotePosition24ArrayListener;
import org.team100.lib.motion.drivetrain.SwerveDriveSubsystem;
import org.team100.lib.util.CameraAngles;

import edu.wpi.first.math.MathUtil;
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
        double f = Math.atan2(m_swerve.getPose().getX(), m_swerve.getPose().getY());
        double l = MathUtil.angleModulus(Math.PI / 2 + f);
        double d = m_swerve.getPose().getRotation().getRadians() - l;
        double no = m_swerve.getPose().getTranslation().getNorm();
        // Translation2d e = m_swerve.getPose().getTranslation();
        // double angle = m_swerve.getPose().getRotation().getRadians();
        // double anglee = e.getAngle().getRadians()-angle;
        double x = no * Math.cos(d);
        switch (Identity.instance) {
            case BETA_BOT:
            case COMP_BOT:
                return m_cameraAngles.getX(m_notePosition24ArrayListener.getY().get());
            case BLANK:
                return x;
            default:
                return x;
        }
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
    public double getAngleToNote() {
        switch (Identity.instance) {
            case BETA_BOT:
            case COMP_BOT:
        return m_cameraAngles.getAngle(m_notePosition24ArrayListener.getX().get(),
                m_notePosition24ArrayListener.getY().get());
                case BLANK:
                return MathUtil.angleModulus(m_swerve.getPose().getTranslation().getAngle().getRadians() + Math.PI);
            default:
                return MathUtil.angleModulus(m_swerve.getPose().getTranslation().getAngle().getRadians() + Math.PI);
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
     *         meters
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
        return new Transform2d(getX(), getY(), new Rotation2d(getAngleToNote()));
    }

    /**
     * @return A field relative Pose2d value of an object in a camera in meters for
     *         translation and radians for rotation
     */
    public Pose2d fieldRelativePose2d() {
        return m_swerve.getPose().transformBy(robotRelativeTransform2d());
    }
}