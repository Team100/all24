package org.team100.lib.commands.drivetrain;

import java.util.function.Supplier;

import org.team100.lib.motion.drivetrain.SpeedLimits;
import org.team100.lib.motion.drivetrain.SwerveDriveSubsystemInterface;
import org.team100.lib.motion.drivetrain.manual.ManualWithHeading;
import org.team100.lib.sensors.HeadingInterface;
import org.team100.lib.telemetry.Telemetry;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;


public class DriveWithHeading extends Command {
    private final Telemetry t = Telemetry.get();
    private final Supplier<Twist2d> m_twistSupplier;
    private final SwerveDriveSubsystemInterface m_drive;

    final ManualWithHeading m_manualWithHeading;

    public DriveWithHeading(
            Supplier<Twist2d> twistSupplier,
            SwerveDriveSubsystemInterface robotDrive,
            HeadingInterface heading,
            SpeedLimits speedLimits,
            Timer timer,
            Supplier<Rotation2d> desiredRotation) {
        m_twistSupplier = twistSupplier;
        m_drive = robotDrive;
        m_manualWithHeading = new ManualWithHeading(speedLimits, heading, timer, desiredRotation);
        if (m_drive.get() != null)
            addRequirements(m_drive.get());
    }

    @Override
    public void initialize() {
        m_manualWithHeading.reset();
    }

    @Override
    public void execute() {
        Pose2d currentPose = m_drive.getPose();
        Twist2d twist1_1 = m_twistSupplier.get();
        Twist2d twistM_S = m_manualWithHeading.apply(currentPose, twist1_1);
        m_drive.driveInFieldCoords(twistM_S);
    }

    @Override
    public void end(boolean interrupted) {
        m_drive.stop();
    }
}
