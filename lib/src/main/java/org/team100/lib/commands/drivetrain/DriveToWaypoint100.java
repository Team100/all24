package org.team100.lib.commands.drivetrain;

import java.util.List;

import org.team100.lib.dashboard.Glassy;
import org.team100.lib.follower.DriveTrajectoryFollower;
import org.team100.lib.logging.Level;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.logging.LoggerFactory.ChassisSpeedsLogger;
import org.team100.lib.motion.drivetrain.SwerveDriveSubsystem;
import org.team100.lib.timing.TimingConstraint;
import org.team100.lib.trajectory.Trajectory100;
import org.team100.lib.trajectory.TrajectoryPlanner;
import org.team100.lib.trajectory.TrajectoryTimeIterator;
import org.team100.lib.trajectory.TrajectoryTimeSampler;
import org.team100.lib.util.DriveUtil;
import org.team100.lib.visualization.TrajectoryVisualization;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;

/**
 * A copy of DriveToWaypoint to explore the new holonomic trajectory classes we
 * cribbed from 254.
 */
public class DriveToWaypoint100 extends Command implements Glassy  {
    // inject these, make them the same as the kinematic limits, inside the
    // trajectory supplier.
    private static final double kMaxVelM_S = 2;
    private static final double kMaxAccelM_S_S = 2;

    private final Pose2d m_goal;
    private final SwerveDriveSubsystem m_swerve;
    private final DriveTrajectoryFollower m_controller;
    private final List<TimingConstraint> m_constraints;

    private final double m_timeBuffer;
    private final TrajectoryVisualization m_viz;
    private final Timer m_timer = new Timer();

    // LOGGERS
    private final ChassisSpeedsLogger m_log_chassis_speeds;

    private Trajectory100 m_trajectory = new Trajectory100();

    public DriveToWaypoint100(
            LoggerFactory parent,
            Pose2d goal,
            SwerveDriveSubsystem drivetrain,
            DriveTrajectoryFollower controller,
            List<TimingConstraint> constraints,
            double timeBuffer,
            TrajectoryVisualization viz) {
        LoggerFactory child = parent.child(this);
        m_log_chassis_speeds = child.chassisSpeedsLogger(Level.TRACE, "chassis speeds");
        m_goal = goal;
        m_swerve = drivetrain;
        m_controller = controller;
        m_constraints = constraints;
        m_timeBuffer = timeBuffer;
        m_viz = viz;
        addRequirements(m_swerve);
    }

    @Override
    public void initialize() {
        final Pose2d start = m_swerve.getState().pose();
        final double startVelocity = 0;
        Pose2d end = m_goal;
        final double endVelocity = 0;
        m_timer.reset();
        m_timer.start();

        List<Pose2d> waypointsM = getWaypoints(start, end);

        List<Rotation2d> headings = List.of(
                start.getRotation(),
                end.getRotation());

        Trajectory100 trajectory = TrajectoryPlanner.generateTrajectory(
                waypointsM,
                headings,
                m_constraints,
                startVelocity,
                endVelocity,
                kMaxVelM_S,
                kMaxAccelM_S_S);
        m_trajectory = trajectory;

        m_viz.setViz(trajectory);

        if (trajectory.isEmpty()) {
            end(false);
            return;
        }
        TrajectoryTimeIterator iter = new TrajectoryTimeIterator(
                new TrajectoryTimeSampler(trajectory));

        m_controller.setTrajectory(iter);
    }

    @Override
    public void execute() {
        double now = Timer.getFPGATimestamp();
        Pose2d currentPose = m_swerve.getState().pose();
        ChassisSpeeds currentSpeed = m_swerve.getState().chassisSpeeds();
        ChassisSpeeds output = m_controller.update(now, currentPose, currentSpeed);
        if (output == null)
            return;

        m_log_chassis_speeds.log(() -> output);
        DriveUtil.checkSpeeds(output);
        m_swerve.setChassisSpeedsNormally(output);
    }

    @Override
    public boolean isFinished() {
        // return m_controller.isDone();
        return m_timer.get() > m_trajectory.getLastPoint().state().getTimeS() + m_timeBuffer;
    }

    @Override
    public void end(boolean interrupted) {
        m_timer.stop();
        m_swerve.stop();
        m_viz.clear();
    }

    ////////////////////////////////////////////////////

    /** Waypoints where the rotation points in the direction of motion. */
    private static List<Pose2d> getWaypoints(Pose2d p0, Pose2d p1) {
        Translation2d t0 = p0.getTranslation();
        Translation2d t1 = p1.getTranslation();
        Rotation2d theta = t1.minus(t0).getAngle();
        return List.of(
                new Pose2d(t0, theta),
                new Pose2d(t1, theta));
    }
}
