package org.team100.frc2023.autonomous;

import org.team100.lib.controller.State100;
import org.team100.lib.motion.drivetrain.SpeedLimits;
import org.team100.lib.motion.drivetrain.SwerveDriveSubsystem;
import org.team100.lib.motion.drivetrain.SwerveState;
import org.team100.lib.profile.MotionProfile;
import org.team100.lib.profile.MotionProfileGenerator;
import org.team100.lib.profile.MotionState;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;

public class DriveToRelative extends Command {
    private final Pose2d relative;
    private final SwerveDriveSubsystem m_robotDrive;
    private final SpeedLimits speedLimits = new SpeedLimits(5, 2, 2, 2);
    private final Timer m_timer;
    private MotionProfile profileX;
    private MotionProfile profileY;
    private MotionProfile profileTheta;

    public DriveToRelative(Pose2d relative, SwerveDriveSubsystem robotDrive) {
        this.relative = relative;
        m_robotDrive = robotDrive;
        m_timer = new Timer();
    }

    @Override
    public void initialize() {
        final Pose2d currentPose = m_robotDrive.getPose();
        profileX = MotionProfileGenerator.generateSimpleMotionProfile(
                new MotionState(currentPose.getX(), 0),
                new MotionState(currentPose.getX() + relative.getX(), 0),
                speedLimits.speedM_S,
                speedLimits.accelM_S2,
                speedLimits.jerkM_S3);

        profileY = MotionProfileGenerator.generateSimpleMotionProfile(
                new MotionState(currentPose.getY(), 0),
                new MotionState(currentPose.getY() + relative.getY(), 0),
                speedLimits.speedM_S,
                speedLimits.accelM_S2,
                speedLimits.jerkM_S3);

        profileTheta = MotionProfileGenerator.generateSimpleMotionProfile(
                new MotionState(currentPose.getRotation().getRadians(), 0),
                new MotionState(MathUtil
                        .angleModulus(currentPose.getRotation().getRadians() + relative.getRotation().getRadians()), 0),
                speedLimits.angleSpeedRad_S,
                speedLimits.angleAccelRad_S2,
                speedLimits.angleJerkRad_S3);

        m_timer.restart();
    }

    @Override
    public void execute() {
        SwerveState desiredState = new SwerveState(new State100(profileX.get(m_timer.get())),
                new State100(profileY.get(m_timer.get())), new State100(profileTheta.get(m_timer.get())));
        m_robotDrive.setDesiredState(desiredState);
    }
}
