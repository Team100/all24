package org.team100.lib.commands.drivetrain;

import java.util.Optional;

import org.team100.lib.controller.drivetrain.HolonomicFieldRelativeController;
import org.team100.lib.dashboard.Glassy;
import org.team100.lib.follower.DriveTrajectoryFollower;
import org.team100.lib.framework.TimedRobot100;
import org.team100.lib.logging.SupplierLogger2;
import org.team100.lib.logging.SupplierLogger2.BooleanSupplierLogger2;
import org.team100.lib.logging.SupplierLogger2.Pose2dLogger;
import org.team100.lib.motion.drivetrain.SwerveDriveSubsystem;
import org.team100.lib.motion.drivetrain.SwerveState;
import org.team100.lib.motion.drivetrain.kinodynamics.FieldRelativeVelocity;
import org.team100.lib.telemetry.Telemetry.Level;
import org.team100.lib.timing.TimedPose;
import org.team100.lib.trajectory.StraightLineTrajectory;
import org.team100.lib.trajectory.Trajectory100;
import org.team100.lib.trajectory.TrajectorySamplePoint;
import org.team100.lib.trajectory.TrajectoryTimeIterator;
import org.team100.lib.trajectory.TrajectoryTimeSampler;
import org.team100.lib.util.Util;
import org.team100.lib.visualization.TrajectoryVisualization;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;

/**
 * Drive from the current state to a field-relative goal.
 * 
 * The trajectory is supplied; the supplier is free to ignore the current state.
 * 
 * The goal rotation is used as the setpoint the entire time, which will put
 * a lot of error into the rotational controller.
 * 
 * If you want a holonomic trajectory follower, try the
 * {@link DriveTrajectoryFollower} classes.
 */
public class DriveToWaypoint3 extends Command implements Glassy {
    /**
     * DriveToWaypoint often appears in sequences, the members of which would want
     * to log into the same key space.
     */
    public static class Log {
        private final Pose2dLogger desired;
        private final BooleanSupplierLogger2 aligned;
        private final Pose2dLogger pose;
        public Log(SupplierLogger2 parent) {
            SupplierLogger2 log = parent.child("DriveToWaypoint3");
            desired = log.pose2dLogger(Level.TRACE, "Desired");
            aligned = log.booleanLogger(Level.TRACE, "Aligned");
            pose = log.pose2dLogger(Level.TRACE, "Pose");
        }
    }

    private final Pose2d m_goal;
    private final SwerveDriveSubsystem m_swerve;
    private final StraightLineTrajectory m_trajectories;
    private final HolonomicFieldRelativeController m_controller;
    private final TrajectoryVisualization m_viz;
    private final Log m_log;

    private Trajectory100 m_trajectory;
    private TrajectoryTimeIterator m_iter;

    /**
     * Trajectory waits until wheels are aligned. If we depend on the setpoint
     * generator to do it, then we're behind the profile timer. After the initial
     * alignment, the steering should be able to keep up with the profile.
     */
    private boolean m_steeringAligned;

    /**
     * @param trajectories function that takes a start and end pose and returns a
     *                     trajectory between them.
     */
    public DriveToWaypoint3(
            Log log,
            Pose2d goal,
            SwerveDriveSubsystem drivetrain,
            StraightLineTrajectory trajectories,
            HolonomicFieldRelativeController controller,
            TrajectoryVisualization viz) {
        m_log = log;
        m_goal = goal;
        m_swerve = drivetrain;
        m_trajectories = trajectories;
        m_controller = controller;
        m_viz = viz;
        addRequirements(m_swerve);

    }

    @Override
    public void initialize() {
        m_controller.reset();
        m_trajectory = m_trajectories.apply(m_swerve.getState(), m_goal);
        m_iter = new TrajectoryTimeIterator(
                new TrajectoryTimeSampler(m_trajectory));
        m_viz.setViz(m_trajectory);
        m_steeringAligned = false;
    }

    @Override
    public void execute() {
        if (m_trajectory == null)
            return;
        SwerveState measurement = m_swerve.getState(); 

        if (m_steeringAligned) {
            Optional<TrajectorySamplePoint> optSamplePoint = m_iter.advance(TimedRobot100.LOOP_PERIOD_S);
            if (optSamplePoint.isEmpty()) {
                Util.warn("broken trajectory, cancelling!");
                cancel(); // this should not happen
                return;
            }
            TrajectorySamplePoint samplePoint = optSamplePoint.get();

            TimedPose desiredState = samplePoint.state();
            m_log.desired.log(() -> desiredState.state().getPose());
            SwerveState reference = SwerveState.fromTimedPose(desiredState);
            FieldRelativeVelocity fieldRelativeTarget = m_controller.calculate(measurement, reference);

            // follow normally
            m_swerve.driveInFieldCoords(fieldRelativeTarget);
        } else {
            // not aligned yet, try aligning by *previewing* next point
            Optional<TrajectorySamplePoint> optSamplePoint = m_iter.preview(TimedRobot100.LOOP_PERIOD_S);
            if (optSamplePoint.isEmpty()) {
                Util.warn("broken trajectory, cancelling!");
                cancel(); // this should not happen
                return;
            }
            TrajectorySamplePoint samplePoint = optSamplePoint.get();

            TimedPose desiredState = samplePoint.state();
            m_log.desired.log(() -> desiredState.state().getPose());
            SwerveState reference = SwerveState.fromTimedPose(desiredState);
            FieldRelativeVelocity fieldRelativeTarget = m_controller.calculate(measurement, reference);

            m_steeringAligned = m_swerve.steerAtRest(fieldRelativeTarget);
        }

        m_log.aligned.log(() -> m_steeringAligned);
        m_log.pose.log(measurement::pose);
    }

    @Override
    public boolean isFinished() {
        if (m_trajectory == null)
            return true;
        return m_iter.isDone() && m_controller.atReference();
    }

    @Override
    public void end(boolean interrupted) {
        m_swerve.stop();
        m_viz.clear();
    }
}
