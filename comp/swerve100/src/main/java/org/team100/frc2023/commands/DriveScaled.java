package org.team100.frc2023.commands;

import java.util.function.Supplier;

import org.team100.lib.commands.DriveUtil;
import org.team100.lib.motion.drivetrain.SpeedLimits;
import org.team100.lib.motion.drivetrain.SwerveDriveSubsystem;
import org.team100.lib.motion.drivetrain.SwerveState;
import org.team100.lib.telemetry.Telemetry;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.wpilibj2.command.Command;

/** Accepts [-1,1] input and scales it to the specified maximum speeds. */
public class DriveScaled extends Command {
    private final Telemetry t = Telemetry.get();

    private final Supplier<Twist2d> m_twistSupplier;
    private final SwerveDriveSubsystem m_robotDrive;
    private final SpeedLimits m_speedLimits;

    public DriveScaled(
            Supplier<Twist2d> twistSupplier,
            SwerveDriveSubsystem robotDrive,
            SpeedLimits speedLimits) {
        m_twistSupplier = twistSupplier;
        m_robotDrive = robotDrive;
        m_speedLimits = speedLimits;
        addRequirements(m_robotDrive);
    }

    @Override
    public void execute() {
        Twist2d twistM_S = DriveUtil.scale(
                m_twistSupplier.get(),
                m_speedLimits.speedM_S,
                m_speedLimits.angleSpeedRad_S);
        
        // System.out.println("TWIIIIISTTTTTT" + twistM_S);
        t.log("/twist and manual/twist x m_s", twistM_S.dx);
        t.log("/twist and manual/twist y m_s", twistM_S.dy);

        Pose2d currentPose = m_robotDrive.getPose();
        SwerveState manualState = SwerveDriveSubsystem.incremental(currentPose, twistM_S);

        t.log("/twist and manual/manual x m_s", manualState.x().x());
        t.log("/twist and manual/manual y m_s", manualState.y().x());

        // System.out.println("MAAAAAAAAAANUALLLLLLLLLL" + manualState);

        m_robotDrive.setDesiredState(manualState);
    }

    @Override
    public void end(boolean interrupted) {
        m_robotDrive.truncate();
    }
}
