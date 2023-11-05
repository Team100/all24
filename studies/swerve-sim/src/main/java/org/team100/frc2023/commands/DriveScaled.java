package org.team100.frc2023.commands;

import java.util.function.Supplier;

import org.team100.lib.commands.DriveUtil;
import org.team100.lib.motion.drivetrain.SpeedLimits;
import org.team100.lib.motion.drivetrain.SwerveState;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Drivetrain;

/** Accepts [-1,1] input and scales it to the specified maximum speeds. */
public class DriveScaled extends Command {
    private final Supplier<Twist2d> m_twistSupplier;
    private final Drivetrain m_robotDrive;
    private final SpeedLimits m_speedLimits;

    public DriveScaled(
            Supplier<Twist2d> twistSupplier,
            Drivetrain robotDrive,
            SpeedLimits speedLimits) {
        m_twistSupplier = twistSupplier;
        m_robotDrive = robotDrive;
        m_speedLimits = speedLimits;
        addRequirements(m_robotDrive);
    }

    @Override
    public void execute() {
        if (m_twistSupplier == null) return;
        Twist2d twistM_S = DriveUtil.scale(
                m_twistSupplier.get(),
                m_speedLimits.speedM_S,
                m_speedLimits.angleSpeedRad_S);

        Pose2d currentPose = m_robotDrive.getPose();
        SwerveState manualState = Drivetrain.incremental(currentPose, twistM_S);
        m_robotDrive.setDesiredState(manualState);
    }

    @Override
    public void end(boolean interrupted) {
        m_robotDrive.truncate();
    }
}