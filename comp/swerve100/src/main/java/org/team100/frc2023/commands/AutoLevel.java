package org.team100.frc2023.commands;

import org.team100.lib.commands.DriveUtil;
import org.team100.lib.motion.drivetrain.SwerveDriveSubsystem;
import org.team100.lib.motion.drivetrain.SwerveState;
import org.team100.lib.motion.drivetrain.kinematics.FrameTransform;
import org.team100.lib.sensors.RedundantGyroInterface;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.wpilibj2.command.Command;

public class AutoLevel extends Command {
    public static class Config {
        public double kMaxSpeed = 4.5;
        public double kMaxRot = 5;
        public double kCruiseSpeed = 1.5;
        /** max speed as a fraction */
        public double kSpeedClamp1_1 = 0.08;
        // TODO: is this unit correct?
        public double kSpeedPerDegree = 0.005;
    }

    private final Config m_config = new Config();
    private final boolean m_reversed;
    private final SwerveDriveSubsystem m_robotDrive;
    private final RedundantGyroInterface m_gyro;
    private final FrameTransform m_chassisSpeedFactory;
    private int count;

    // TODO: what is "reversed" for?
    public AutoLevel(
            boolean reversed,
            SwerveDriveSubsystem robotDrive,
            RedundantGyroInterface gyro,
            FrameTransform chassisSpeedFactory) {
        m_reversed = reversed;
        m_robotDrive = robotDrive;
        m_gyro = gyro;
        m_chassisSpeedFactory = chassisSpeedFactory;
        addRequirements(m_robotDrive);
    }

    @Override
    public void initialize() {
        count = 0;
    }

    @Override
    public void execute() {
        Rotation2d rot = m_robotDrive.getPose().getRotation();

        double Roll = m_gyro.getRedundantRoll();
        double Pitch = m_gyro.getRedundantPitch();
        double ySpeed = MathUtil.clamp(m_config.kSpeedPerDegree * Roll, -m_config.kSpeedClamp1_1,
                m_config.kSpeedClamp1_1);
        double xSpeed = MathUtil.clamp(m_config.kSpeedPerDegree * Pitch, -m_config.kSpeedClamp1_1,
                m_config.kSpeedClamp1_1);

        if (m_reversed) {
            if (Math.abs(Roll) > 2.5 || Math.abs(Pitch) > 2.5) {
                count = 0;

                Twist2d twist = new Twist2d(xSpeed, ySpeed, 0);
                Twist2d twistM_S = DriveUtil.scale(twist, m_config.kMaxSpeed, m_config.kMaxRot);
                Twist2d fieldRelative = m_chassisSpeedFactory.toFieldRelativeSpeeds(
                        twistM_S.dx, twistM_S.dy, twistM_S.dtheta, rot);

                Pose2d currentPose = m_robotDrive.getPose();
                SwerveState manualState = SwerveDriveSubsystem.incremental(currentPose, fieldRelative);
                m_robotDrive.setDesiredState(manualState);
            } else {
                count++;
            }
        } else {
            if (m_robotDrive.getPose().getX() >= 3.277) {
                if (Math.abs(Roll) > 2.5 || Math.abs(Pitch) > 2.5) {
                    count = 0;

                    Twist2d twist = new Twist2d(xSpeed, -ySpeed, 0);
                    Twist2d twistM_S = DriveUtil.scale(twist, m_config.kMaxSpeed, m_config.kMaxRot);
                    Twist2d fieldRelative = m_chassisSpeedFactory.toFieldRelativeSpeeds(
                            twistM_S.dx, twistM_S.dy, twistM_S.dtheta, rot);

                    Pose2d currentPose = m_robotDrive.getPose();
                    SwerveState manualState = SwerveDriveSubsystem.incremental(currentPose, fieldRelative);
                    m_robotDrive.setDesiredState(manualState);
                } else {
                    count++;
                }
            } else {
                Twist2d twistM_S = new Twist2d(m_config.kCruiseSpeed, 0, 0);

                Pose2d currentPose = m_robotDrive.getPose();
                SwerveState manualState = SwerveDriveSubsystem.incremental(currentPose, twistM_S);
                m_robotDrive.setDesiredState(manualState);
            }
        }
    }

    @Override
    public boolean isFinished() {
        return count >= 20;
    }

    @Override
    public void end(boolean interrupted) {
        m_robotDrive.truncate();
    }
}
