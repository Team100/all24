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
            return fieldRelativePose2d().getX();
    }

    /**
     * @return A robot relative angle in radians to the note
     */
    public double getAngleToNote() {
        return m_cameraAngles.getAngle(m_notePosition24ArrayListener.getX().get(),
                m_notePosition24ArrayListener.getY().get());
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
            return fieldRelativePose2d().getY();
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
        switch (Identity.instance) {
            case BETA_BOT:
                    return m_swerve.getPose().transformBy(robotRelativeTransform2d());
            default:
            //I want the sim to go to a pose similar to how it would attack a note, so I am putting the angle of attack equal to 0,0 with the angle to that point, is there a better way to do this?
                return new Pose2d(0,0,new Rotation2d(Math.atan2(-1.0 * m_swerve.getPose().getY(),-1.0 * m_swerve.getPose().getX())));
        }
    }
}