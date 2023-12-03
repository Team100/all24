package org.team100.lib.commands.drivetrain;

import org.team100.lib.config.Identity;
import org.team100.lib.controller.DriveControllers;
import org.team100.lib.controller.DriveControllersFactory;
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

public class DriveToRelative extends Command {
    private final Pose2d relative;
    private final SwerveDriveSubsystemInterface m_robotDrive;
    private final SpeedLimits speedLimits = new SpeedLimits(5, 2, 2, 2);
    private final Timer m_timer;
    private final HolonomicDriveController3 m_controller;
    private MotionProfile profileX;
    private MotionProfile profileY;
    private MotionProfile profileTheta;

    public DriveToRelative(Pose2d relative, SwerveDriveSubsystemInterface robotDrive) {
        this.relative = relative;
        m_robotDrive = robotDrive;
        m_timer = new Timer();
        Identity identity = Identity.get();

        DriveControllers controllers = new DriveControllersFactory().get(identity);

        m_controller = new HolonomicDriveController3(controllers);
        m_controller.setTolerance(0.1, 0.1, 0.1, 0.1);
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
                new MotionState(MathUtil.angleModulus(currentPose.getRotation().getRadians()
                        + relative.getRotation().getRadians()), 0),
                speedLimits.angleSpeedRad_S,
                speedLimits.angleAccelRad_S2,
                speedLimits.angleJerkRad_S3);

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
