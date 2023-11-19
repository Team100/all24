package org.team100.lib.commands;

import java.util.function.Supplier;

import org.team100.lib.motion.drivetrain.SpeedLimits;
import org.team100.lib.motion.drivetrain.SwerveDriveSubsystem;
import org.team100.lib.motion.drivetrain.manual.ManualChassisSpeeds;
import org.team100.lib.motion.drivetrain.manual.ManualFieldRelativeSpeeds;
import org.team100.lib.motion.drivetrain.manual.ManualModuleStates;

import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.Command;

/** Uses a Sendable Chooser */
public class DriveManually extends Command {
    private final SwerveDriveSubsystem m_drive;
    private final ManualModuleStates m_manualModuleStates;
    private final ManualChassisSpeeds m_manualChassisSpeeds;
    private final ManualFieldRelativeSpeeds m_manualFieldRelativeSpeeds;
    private final ManualMode m_mode;

    public DriveManually(
            ManualMode mode,
            Supplier<Twist2d> twistSupplier,
            SwerveDriveSubsystem robotDrive,
            SpeedLimits speedLimits) {
        m_mode = mode;
        m_drive = robotDrive;
        m_manualModuleStates = new ManualModuleStates(twistSupplier, speedLimits);
        m_manualChassisSpeeds = new ManualChassisSpeeds(twistSupplier, speedLimits);
        m_manualFieldRelativeSpeeds = new ManualFieldRelativeSpeeds(twistSupplier, speedLimits);
        addRequirements(m_drive);
    }

    @Override
    public void execute() {
        ManualMode.Mode manualMode = m_mode.getSelected();
        if (manualMode == null) {
            return;
        }

        switch (manualMode) {
            case MODULE_STATE:
                SwerveModuleState[] states = m_manualModuleStates.get();
                m_drive.setRawModuleStates(states);
                break;
            case ROBOT_RELATIVE_CHASSIS_SPEED:
                ChassisSpeeds speeds = m_manualChassisSpeeds.get();
                m_drive.setChassisSpeeds(speeds);
                break;
            case FIELD_RELATIVE_TWIST:
                Twist2d twist = m_manualFieldRelativeSpeeds.get();
                m_drive.driveInFieldCoords(twist);
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
