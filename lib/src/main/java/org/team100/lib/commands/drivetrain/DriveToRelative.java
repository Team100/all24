package org.team100.lib.commands.drivetrain;

import org.team100.lib.controller.HolonomicDriveController3;
import org.team100.lib.controller.State100;
import org.team100.lib.motion.drivetrain.SpeedLimits;
import org.team100.lib.motion.drivetrain.SwerveDriveSubsystemInterface;
import org.team100.lib.motion.drivetrain.SwerveState;
import org.team100.lib.profile.MotionProfile;
import org.team100.lib.profile.MotionProfileGenerator;
import org.team100.lib.profile.MotionState;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;

/**
 * Drive to a pose relative to the current pose.
 * 
 * Uses three independent MotionProfiles: the resulting path will not generally
 * be exactly a straight line, but it will be pretty close.
 * 
 * TODO: perhaps the relative pose should be a Transform2d instead.
 */
public class DriveToRelative extends Command {
    private final Pose2d relative;
    private final SwerveDriveSubsystemInterface m_robotDrive;
    private final SpeedLimits m_speedLimits;
    private final Timer m_timer;
    private final HolonomicDriveController3 m_controller;

    private MotionProfile profileX;
    private MotionProfile profileY;
    private MotionProfile profileTheta;

    public DriveToRelative(
            Pose2d relative,
            SpeedLimits speedLimits,
            SwerveDriveSubsystemInterface robotDrive,
            HolonomicDriveController3 controller) {
        this.relative = relative;
        m_speedLimits = speedLimits;
        m_robotDrive = robotDrive;
        m_timer = new Timer();
        m_controller = controller;
    }

    @Override
    public void initialize() {
        m_controller.reset();
        final Pose2d currentPose = m_robotDrive.getPose();
        profileX = MotionProfileGenerator.generateSimpleMotionProfile(
                new MotionState(currentPose.getX(), 0),
                new MotionState(currentPose.getX() + relative.getX(), 0),
                m_speedLimits.speedM_S,
                m_speedLimits.accelM_S2,
                m_speedLimits.jerkM_S3);

        profileY = MotionProfileGenerator.generateSimpleMotionProfile(
                new MotionState(currentPose.getY(), 0),
                new MotionState(currentPose.getY() + relative.getY(), 0),
                m_speedLimits.speedM_S,
                m_speedLimits.accelM_S2,
                m_speedLimits.jerkM_S3);

        profileTheta = MotionProfileGenerator.generateSimpleMotionProfile(
                new MotionState(currentPose.getRotation().getRadians(), 0),
                new MotionState(
                        MathUtil.angleModulus(
                                currentPose.getRotation().getRadians()
                                        + relative.getRotation().getRadians()),
                        0),
                m_speedLimits.angleSpeedRad_S,
                m_speedLimits.angleAccelRad_S2,
                m_speedLimits.angleJerkRad_S3);

        m_timer.restart();
    }

    @Override
    public void execute() {
        Pose2d currentPose = m_robotDrive.getPose();
        SwerveState reference = new SwerveState(
                new State100(profileX.get(m_timer.get())),
                new State100(profileY.get(m_timer.get())),
                new State100(profileTheta.get(m_timer.get())));
        Twist2d fieldRelativeTarget = m_controller.calculate(currentPose, reference);
        m_robotDrive.driveInFieldCoords(fieldRelativeTarget);
    }

    @Override
    public boolean isFinished() {
        return m_timer.get() > duration() && m_controller.atReference();
    }

    @Override
    public void end(boolean interrupted) {
        m_robotDrive.stop();
    }

    private double duration() {
        return Math.max(Math.max(profileX.duration(), profileY.duration()), profileTheta.duration());
    }
}
