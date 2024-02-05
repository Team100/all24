package org.team100.lib.commands.drivetrain;

import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

import org.team100.lib.commands.Command100;
import org.team100.lib.motion.drivetrain.SwerveDriveSubsystem;
import org.team100.lib.motion.drivetrain.SwerveState;
import org.team100.lib.motion.drivetrain.kinodynamics.SwerveKinodynamics;
import org.team100.lib.motion.drivetrain.manual.ManualWithNoteRotation;
import org.team100.lib.motion.drivetrain.manual.ManualChassisSpeeds;
import org.team100.lib.motion.drivetrain.manual.ManualFieldRelativeSpeeds;
import org.team100.lib.motion.drivetrain.manual.ManualWithHeading;
import org.team100.lib.motion.drivetrain.manual.ManualWithTargetLock;
import org.team100.lib.motion.drivetrain.manual.SimpleManualModuleStates;
import org.team100.lib.sensors.HeadingInterface;
import org.team100.lib.swerve.SwerveSetpoint;
import org.team100.lib.util.CameraAngles;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;

/**
 * Manual drivetrain control.
 * 
 * Provides four manual control modes:
 * 
 * -- raw module state
 * -- robot-relative
 * -- field-relative
 * -- field-relative with rotation control
 * 
 * Use the mode supplier to choose which mode to use, e.g. using a Sendable
 * Chooser.
 */
public class DriveManually extends Command100 {
    private final Supplier<ManualMode.Mode> m_mode;
    /**
     * Velocity control in control units, [-1,1] on all axes. This needs to be
     * mapped to a feasible velocity control as early as possible.
     */
    private final Supplier<Twist2d> m_twistSupplier;
    private final SwerveDriveSubsystem m_drive;
    private final SimpleManualModuleStates m_manualModuleStates;
    private final ManualChassisSpeeds m_manualChassisSpeeds;
    private final ManualFieldRelativeSpeeds m_manualFieldRelativeSpeeds;
    private final ManualWithHeading m_manualWithHeading;
    private final ManualWithTargetLock m_manualWithTargetLock;
    private final ManualWithNoteRotation m_driveWithNoteRotation;

    ManualMode.Mode currentManualMode = null;

    public DriveManually(
            Supplier<ManualMode.Mode> mode,
            Supplier<Twist2d> twistSupplier,
            SwerveDriveSubsystem robotDrive,
            HeadingInterface heading,
            SwerveKinodynamics swerveKinodynamics,
            Supplier<Rotation2d> desiredRotation,
            PIDController thetaController,
            PIDController omegaController,
            Supplier<Translation2d> target,
            BooleanSupplier trigger,
            CameraAngles noteCamera) {
        m_mode = mode;
        m_twistSupplier = twistSupplier;
        m_drive = robotDrive;
        m_manualModuleStates = new SimpleManualModuleStates(m_name, swerveKinodynamics);
        m_manualChassisSpeeds = new ManualChassisSpeeds(m_name, swerveKinodynamics);
        m_manualFieldRelativeSpeeds = new ManualFieldRelativeSpeeds(m_name, swerveKinodynamics);
        m_manualWithHeading = new ManualWithHeading(
                m_name,
                swerveKinodynamics,
                heading,
                desiredRotation,
                thetaController,
                omegaController);
        m_manualWithTargetLock = new ManualWithTargetLock(
                m_name,
                swerveKinodynamics,
                heading,
                target,
                thetaController,
                omegaController,
                trigger);
        m_driveWithNoteRotation = new ManualWithNoteRotation(
                m_name,
                swerveKinodynamics,
                thetaController,
                noteCamera);
        addRequirements(m_drive);
    }

    @Override
    public void initialize100() {
        // the setpoint generator remembers what it was doing before, but it might be
        // interrupted by some other command, so when we start, we have to tell it what
        // the real previous setpoint is.
        // Note this is not necessarily "at rest," because we might start driving
        // manually while the robot is moving.
        ChassisSpeeds currentSpeeds = m_drive.speeds(0.02);
        SwerveModuleState[] currentStates = m_drive.moduleStates();
        SwerveSetpoint setpoint = new SwerveSetpoint(currentSpeeds, currentStates);
        m_drive.resetSetpoint(setpoint);
    }

    @Override
    public void execute100(double dt) {
        ManualMode.Mode manualMode = m_mode.get();
        if (manualMode == null) {
            return;
        }

        if (manualMode != currentManualMode) {
            currentManualMode = manualMode;
            // there's state in there we'd like to forget
            m_manualWithHeading.reset(m_drive.getPose());
            m_manualWithTargetLock.reset(m_drive.getPose());
        }

        // input in [-1,1] control units
        Twist2d input = m_twistSupplier.get();
        SwerveState state = m_drive.getState();
        Pose2d currentPose = state.pose();

        /**
         * None of these transformers pay attention to feasibility. Feasibility is
         * enforced by the drivetrain setpoint generator and desaturator.
         */
        switch (manualMode) {
            case MODULE_STATE:
                m_drive.setRawModuleStates(
                        m_manualModuleStates.apply(input));
                break;
            case ROBOT_RELATIVE_CHASSIS_SPEED:
                m_drive.setChassisSpeeds(
                        m_manualChassisSpeeds.apply(input), dt);
                break;
            case ROBOT_RELATIVE_FACING_NOTE:
                m_drive.setChassisSpeeds(
                        m_driveWithNoteRotation.apply(input), dt);
                break;
            case FIELD_RELATIVE_TWIST:
                m_drive.driveInFieldCoords(
                        m_manualFieldRelativeSpeeds.apply(input), dt);
                break;
            case SNAPS:
                m_drive.driveInFieldCoords(
                        m_manualWithHeading.apply(currentPose, input), dt);
                break;
            case LOCKED:
                m_drive.driveInFieldCoords(
                        m_manualWithTargetLock.apply(state, input), dt);
                break;
            default:
                // do nothing
                break;
        }
    }

    @Override
    public void end(boolean interrupted) {
        m_drive.stop();
    }
}
