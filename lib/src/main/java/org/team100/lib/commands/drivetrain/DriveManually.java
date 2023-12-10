package org.team100.lib.commands.drivetrain;

import java.util.function.Function;
import java.util.function.Supplier;

import org.team100.lib.motion.drivetrain.SpeedLimits;
import org.team100.lib.motion.drivetrain.SwerveDriveSubsystemInterface;
import org.team100.lib.motion.drivetrain.manual.ManualChassisSpeeds;
import org.team100.lib.motion.drivetrain.manual.ManualFieldRelativeSpeeds;
import org.team100.lib.motion.drivetrain.manual.SimpleManualModuleStates;

import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.Command;

/** Uses a Sendable Chooser */
public class DriveManually extends Command {
    private final Supplier<ManualMode.Mode> m_mode;
    private final Supplier<Twist2d> m_twistSupplier;
    private final SwerveDriveSubsystemInterface m_drive;

    private final Function<Twist2d, SwerveModuleState[]> m_manualModuleStates;
    private final Function<Twist2d, ChassisSpeeds> m_manualChassisSpeeds;
    private final Function<Twist2d, Twist2d> m_manualFieldRelativeSpeeds;

    public DriveManually(
            Supplier<ManualMode.Mode> mode,
            Supplier<Twist2d> twistSupplier,
            SwerveDriveSubsystemInterface robotDrive,
            SpeedLimits speedLimits) {
        m_mode = mode;
        m_twistSupplier = twistSupplier;
        m_drive = robotDrive;
        // m_manualModuleStates = new ManualModuleStates(twistSupplier, speedLimits);
        m_manualModuleStates = new SimpleManualModuleStates(speedLimits);
        m_manualChassisSpeeds = new ManualChassisSpeeds(speedLimits);
        m_manualFieldRelativeSpeeds = new ManualFieldRelativeSpeeds(speedLimits);
        if (m_drive.get() != null)
            addRequirements(m_drive.get());
    }

    @Override
    public void execute() {
        ManualMode.Mode manualMode = m_mode.get();
        if (manualMode == null) {
            return;
        }
        Twist2d input = m_twistSupplier.get();

        switch (manualMode) {
            case MODULE_STATE:
                SwerveModuleState[] states = m_manualModuleStates.apply(input);
                m_drive.setRawModuleStates(states);
                break;
            case ROBOT_RELATIVE_CHASSIS_SPEED:
                ChassisSpeeds speeds = m_manualChassisSpeeds.apply(input);
                m_drive.setChassisSpeeds(speeds);
                break;
            case FIELD_RELATIVE_TWIST:
                Twist2d twist = m_manualFieldRelativeSpeeds.apply(input);
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
