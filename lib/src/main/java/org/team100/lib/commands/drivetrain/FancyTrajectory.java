package org.team100.lib.commands.drivetrain;

import java.util.List;

import org.team100.lib.commands.Command100;
import org.team100.lib.controller.DriveMotionController;
import org.team100.lib.controller.DrivePIDFController;
import org.team100.lib.geometry.GeometryUtil;
import org.team100.lib.motion.drivetrain.SwerveDriveSubsystem;
import org.team100.lib.telemetry.Telemetry;
import org.team100.lib.telemetry.Telemetry.Level;
import org.team100.lib.timing.CentripetalAccelerationConstraint;
import org.team100.lib.timing.TimingConstraint;
import org.team100.lib.trajectory.Trajectory100;
import org.team100.lib.trajectory.TrajectoryPlanner;
import org.team100.lib.trajectory.TrajectoryTimeIterator;
import org.team100.lib.trajectory.TrajectoryTimeSampler;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;

/**
 * Follow a fixed trajectory, using the new 254-derived trajectory and follower
 * types.
 * 
 * This is an experiment.
 */
public class FancyTrajectory extends Command100 {
    private static final double kMaxVelM_S = 4;
    private static final double kMaxAccelM_S_S = 2;

    private final Telemetry t = Telemetry.get();

    private final SwerveDriveSubsystem m_robotDrive;
    private final DriveMotionController m_controller;
    private final TrajectoryPlanner m_planner;

    public FancyTrajectory(
            SwerveDriveSubsystem robotDrive,
            TrajectoryPlanner planner) {
        m_robotDrive = robotDrive;
        m_controller = new DrivePIDFController(false);
        m_planner = planner;
        addRequirements(m_robotDrive);
    }

    @Override
    public void initialize100() {
        List<Pose2d> waypointsM = List.of(
                new Pose2d(0, 0, Rotation2d.fromDegrees(90)),
                new Pose2d(80, 80, Rotation2d.fromDegrees(0)));
        // while turning 180
        List<Rotation2d> headings = List.of(
                GeometryUtil.fromDegrees(0),
                GeometryUtil.fromDegrees(0));
        // these don't actually do anything.
        List<TimingConstraint> constraints = List.of(
                new CentripetalAccelerationConstraint(60));

        double start_vel = 0;
        double end_vel = 0;
        // there's a bug in here; it doesn't use the constraints, nor the voltage.
        Trajectory100 trajectory = m_planner.generateTrajectory(
                false,
                waypointsM,
                headings,
                constraints,
                start_vel,
                end_vel,
                kMaxVelM_S,
                kMaxAccelM_S_S);

        TrajectoryTimeIterator iter = new TrajectoryTimeIterator(new TrajectoryTimeSampler(trajectory));

        m_controller.setTrajectory(iter);
    }

    @Override
    public void execute100(double dt) {
        final double now = Timer.getFPGATimestamp();
        Pose2d currentPose = m_robotDrive.getPose();
        ChassisSpeeds currentSpeed = m_robotDrive.speeds();
        Twist2d velocity = GeometryUtil.toTwist2d(currentSpeed);
        ChassisSpeeds output = m_controller.update(now, currentPose, velocity);
        t.log(Level.DEBUG, "/fancy trajectory/chassis speeds", output);
        m_robotDrive.setChassisSpeeds(output, dt);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
