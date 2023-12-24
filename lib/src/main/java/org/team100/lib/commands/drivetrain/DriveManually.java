package org.team100.lib.commands.drivetrain;

import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

import org.team100.lib.motion.drivetrain.SwerveDriveSubsystemInterface;
import org.team100.lib.motion.drivetrain.kinodynamics.SwerveKinodynamics;
import org.team100.lib.motion.drivetrain.manual.ManualChassisSpeeds;
import org.team100.lib.motion.drivetrain.manual.ManualFieldRelativeSpeeds;
import org.team100.lib.motion.drivetrain.manual.ManualWithHeading;
import org.team100.lib.motion.drivetrain.manual.ManualWithTargetLock;
import org.team100.lib.motion.drivetrain.manual.SimpleManualModuleStates;
import org.team100.lib.sensors.HeadingInterface;
import org.team100.lib.swerve.SwerveSetpoint;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.Command;

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
public class DriveManually extends Command {
    private final Supplier<ManualMode.Mode> m_mode;
    private final Supplier<Twist2d> m_twistSupplier;
    private final SwerveDriveSubsystemInterface m_drive;

    private final SimpleManualModuleStates m_manualModuleStates;
    private final ManualChassisSpeeds m_manualChassisSpeeds;
    private final ManualFieldRelativeSpeeds m_manualFieldRelativeSpeeds;
    private final ManualWithHeading m_manualWithHeading;
    private final ManualWithTargetLock m_manualWithTargetLock;

    ManualMode.Mode currentManualMode = null;

    public DriveManually(
            Supplier<ManualMode.Mode> mode,
            Supplier<Twist2d> twistSupplier,
            SwerveDriveSubsystemInterface robotDrive,
            HeadingInterface heading,
            SwerveKinodynamics speedLimits,
            Supplier<Rotation2d> desiredRotation,
            PIDController thetaController,
            Supplier<Translation2d> target,
            BooleanSupplier trigger) {
        m_mode = mode;
        m_twistSupplier = twistSupplier;
        m_drive = robotDrive;
        m_manualModuleStates = new SimpleManualModuleStates(speedLimits);
        m_manualChassisSpeeds = new ManualChassisSpeeds(speedLimits);
        m_manualFieldRelativeSpeeds = new ManualFieldRelativeSpeeds(speedLimits);
        m_manualWithHeading = new ManualWithHeading(
                speedLimits,
                heading,
                desiredRotation,
                thetaController);
        m_manualWithTargetLock = new ManualWithTargetLock(
                speedLimits,
                target,
                thetaController,
                trigger);
        if (m_drive.get() != null)
            addRequirements(m_drive.get());
    }

    @Override
    public void initialize() {
        // the setpoint generator remembers what it was doing before, but it might be
        // interrupted by some other command, so when we start, we have to tell it what
        // the real previous setpoint is.
        // Note this is not necessarily "at rest," because we might start driving
        // manually while the robot is moving.
        ChassisSpeeds currentSpeeds = m_drive.speeds();
        SwerveModuleState[] currentStates = m_drive.moduleStates();
        SwerveSetpoint setpoint = new SwerveSetpoint(currentSpeeds, currentStates);
        m_drive.resetSetpoint(setpoint);
    }

    @Override
    public void execute() {
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

        Twist2d input = m_twistSupplier.get();
        Pose2d currentPose = m_drive.getPose();

        switch (manualMode) {
            case MODULE_STATE:
                m_drive.setRawModuleStates(
                        m_manualModuleStates.apply(input));
                break;
            case ROBOT_RELATIVE_CHASSIS_SPEED:
                m_drive.setChassisSpeeds(
                        m_manualChassisSpeeds.apply(input));
                break;
            case FIELD_RELATIVE_TWIST:
                m_drive.driveInFieldCoords(
                        m_manualFieldRelativeSpeeds.apply(input));
                break;
            case SNAPS:
                m_drive.driveInFieldCoords(
                        m_manualWithHeading.apply(currentPose, input));
                break;
            case LOCKED:
                m_drive.driveInFieldCoords(
                        m_manualWithTargetLock.apply(currentPose, input));
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
