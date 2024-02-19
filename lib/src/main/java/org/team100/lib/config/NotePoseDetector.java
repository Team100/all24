package org.team100.lib.config;

import java.util.Optional;

import org.team100.lib.localization.NotePosition24ArrayListener;
import org.team100.lib.motion.drivetrain.SwerveDriveSubsystem;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;

public class NotePoseDetector {
    private final SwerveDriveSubsystem m_swerve;
    private final NotePosition24ArrayListener m_notePosition24ArrayListener;

    /**
     * @param notePosition24ArrayListener Class which gets the x and a values of the
     *                                    object detected from the PI
     * @param swerve                      The swerve drivetrain
     */
    public NotePoseDetector(
            NotePosition24ArrayListener notePosition24ArrayListener,
            SwerveDriveSubsystem swerve) {
        m_notePosition24ArrayListener = notePosition24ArrayListener;
        m_swerve = swerve;
    }

    /**
     * @return A field relative angle in radians to the note
     */
    public Optional<Rotation2d> fieldRelativeAngleToNote(Rotation2d angle) {
        switch (Identity.instance) {
            case BETA_BOT:
            case COMP_BOT:
            case BLANK:
                if (fieldRelativePose2d(angle).isPresent()) {
                    return Optional.of(fieldRelativePose2d(angle).get().getRotation());
                }
                return Optional.empty();
            // return
            // FieldRelativeTranslation2d().minus(m_swerve.getPose().getTranslation()).getAngle();
            default:
                return Optional.of(
                        FieldRelativeTranslation2d(angle).get().minus(m_swerve.getPose().getTranslation()).getAngle());
        }
    }

    /**
     * @return A robot relative translational value of an object in a camera in
     *         meters, rot value is bearing
     */
    public Optional<Translation2d> RobotRelativeTranslation2d() {
        if (m_notePosition24ArrayListener.getTranslation2d()[0].isPresent()) {
            return m_notePosition24ArrayListener.getTranslation2d()[0];
        }
        return Optional.empty();
    }

    /**
     * @return A field relative translational value of an object in a camera in
     *         meters
     */
    public Optional<Translation2d> FieldRelativeTranslation2d(Rotation2d angle) {
        switch (Identity.instance) {
            case BETA_BOT:
            case COMP_BOT:
            case BLANK:
                if (fieldRelativePose2d(angle).isPresent()) {
                    return Optional.of(fieldRelativePose2d(angle).get().getTranslation());
                }
                return Optional.empty();
            default:
                return Optional.of(new Translation2d());
        }
    }

    /**
     * @return A field relative Pose2d value of an object in a camera in meters for
     *         translation and radians for rotation
     */
    public Optional<Pose2d> fieldRelativePose2d(Rotation2d angle) {
        switch (Identity.instance) {
            case BETA_BOT:
            case COMP_BOT:
            case BLANK:
                if (RobotRelativeTranslation2d().isPresent()) {
                    return Optional.of(
                            m_swerve.getPose().transformBy(new Transform2d(RobotRelativeTranslation2d().get(), angle)));
                }
                return Optional.empty();
            default:
                return Optional
                        .of(new Pose2d(FieldRelativeTranslation2d(angle).get(), fieldRelativeAngleToNote(angle).get()));
        }
    }
}