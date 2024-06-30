package org.team100.lib.commands.drivetrain;

import java.util.List;

import org.team100.lib.commands.Command100;
import org.team100.lib.controller.DriveMotionController;
import org.team100.lib.motion.drivetrain.SwerveDriveSubsystem;
import org.team100.lib.telemetry.Telemetry;
import org.team100.lib.telemetry.Telemetry.Level;
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

/**
 * A copy of DriveToWaypoint to explore the new holonomic trajectory classes we
 * cribbed from 254.
 */
public class DriveToWaypoint100 extends Command100 {
    // inject these, make them the same as the kinematic limits, inside the
    // trajectory supplier.
    private static final double kMaxVelM_S = 2;
    private static final double kMaxAccelM_S_S = 2;
    private static final Telemetry t = Telemetry.get();

    private final Pose2d m_goal;
    private final SwerveDriveSubsystem m_swerve;
    private final DriveMotionController m_controller;
    private final List<TimingConstraint> m_constraints;

    private final double m_timeBuffer;
    private final Timer m_timer = new Timer();

    private Trajectory100 m_trajectory = new Trajectory100();

    public DriveToWaypoint100(
            Pose2d goal,
            SwerveDriveSubsystem drivetrain,
            DriveMotionController controller,
            List<TimingConstraint> constraints,
            double timeBuffer) {
        m_goal = goal;
        m_swerve = drivetrain;
        m_controller = controller;
        m_constraints = constraints;
        m_timeBuffer = timeBuffer;
        addRequirements(m_swerve);
    }

    @Override
    public void initialize100() {
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

        TrajectoryVisualization.setViz(trajectory);

        if (trajectory.isEmpty()) {
            end(false);
            return;
        }
        TrajectoryTimeIterator iter = new TrajectoryTimeIterator(
                new TrajectoryTimeSampler(trajectory));

        m_controller.setTrajectory(iter);
    }

    @Override
    public void execute100(double dt) {
        double now = Timer.getFPGATimestamp();
        Pose2d currentPose = m_swerve.getState().pose();
        ChassisSpeeds currentSpeed = m_swerve.getState().chassisSpeeds();
        ChassisSpeeds output = m_controller.update(now, currentPose, currentSpeed);
        if (output == null)
            return;

        t.log(Level.DEBUG, m_name, "chassis speeds", output);
        DriveUtil.checkSpeeds(output);
        m_swerve.setChassisSpeedsNormally(output, dt);
    }

    @Override
    public boolean isFinished() {
        // return m_controller.isDone();
        return m_timer.get() > m_trajectory.getLastPoint().state().getTimeS() + m_timeBuffer;
    }

    @Override
    public void end100(boolean interrupted) {
        m_timer.stop();
        m_swerve.stop();
        TrajectoryVisualization.clear();
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
