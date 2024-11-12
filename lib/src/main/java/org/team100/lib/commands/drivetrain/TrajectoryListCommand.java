package org.team100.lib.commands.drivetrain;

import java.util.Iterator;
import java.util.List;
import java.util.Optional;
import java.util.function.Function;

import org.team100.lib.controller.drivetrain.HolonomicFieldRelativeController;
import org.team100.lib.dashboard.Glassy;
import org.team100.lib.follower.DriveTrajectoryFollower;
import org.team100.lib.framework.TimedRobot100;
import org.team100.lib.logging.Level;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.logging.LoggerFactory.SwerveStateLogger;
import org.team100.lib.motion.drivetrain.SwerveDriveSubsystem;
import org.team100.lib.motion.drivetrain.SwerveState;
import org.team100.lib.motion.drivetrain.kinodynamics.FieldRelativeVelocity;
import org.team100.lib.timing.TimedPose;
import org.team100.lib.trajectory.Trajectory100;
import org.team100.lib.trajectory.TrajectorySamplePoint;
import org.team100.lib.trajectory.TrajectoryTimeIterator;
import org.team100.lib.trajectory.TrajectoryTimeSampler;
import org.team100.lib.util.Util;
import org.team100.lib.visualization.TrajectoryVisualization;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;

/**
 * Follow a list of trajectories.
 * 
 * The list can be relative to the current pose.
 * 
 * This just holds the starting rotation. If you want a holonomic trajectory
 * follower, try the {@link DriveTrajectoryFollower} classes.
 */
public class TrajectoryListCommand extends Command implements Glassy {
    private final SwerveDriveSubsystem m_swerve;
    private final HolonomicFieldRelativeController m_controller;
    private final Function<Pose2d, List<Trajectory100>> m_trajectories;
    private final TrajectoryVisualization m_viz;
    // LOGGERS
    private final SwerveStateLogger m_log_reference;

    private Iterator<Trajectory100> m_trajectoryIter;
    private TrajectoryTimeIterator m_iter;
    private boolean m_aligned;
    private boolean done;

    public TrajectoryListCommand(
            LoggerFactory parent,
            SwerveDriveSubsystem swerve,
            HolonomicFieldRelativeController controller,
            Function<Pose2d, List<Trajectory100>> trajectories,
            TrajectoryVisualization viz) {
        LoggerFactory child = parent.child(this);
        m_swerve = swerve;
        m_controller = controller;
        m_trajectories = trajectories;
        m_viz = viz;
        addRequirements(m_swerve);
        m_log_reference = child.swerveStateLogger(Level.TRACE, "reference");
    }

    @Override
    public void initialize() {
        m_controller.reset();
        Pose2d currentPose = m_swerve.getPose();
        m_trajectoryIter = m_trajectories.apply(currentPose).iterator();
        m_iter = null;
        done = false;
        m_aligned = false;
    }

    @Override
    public void execute() {
        if (m_iter == null || m_iter.isDone()) {
            // get the next trajectory
            if (m_trajectoryIter.hasNext()) {
                Trajectory100 m_currentTrajectory = m_trajectoryIter.next();
                m_iter = new TrajectoryTimeIterator(
                        new TrajectoryTimeSampler(m_currentTrajectory));
                m_viz.setViz(m_currentTrajectory);
                m_aligned = false;
            } else {
                done = true;
                return;
            }
        }

        // now there is a trajectory to follow

        if (m_aligned) {
            Optional<TrajectorySamplePoint> optSamplePoint = m_iter.advance(TimedRobot100.LOOP_PERIOD_S);
            if (optSamplePoint.isEmpty()) {
                Util.warn("broken trajectory, cancelling!");
                cancel(); // this should not happen
                return;
            }
            TrajectorySamplePoint samplePoint = optSamplePoint.get();
            TimedPose desiredState = samplePoint.state();

            SwerveState reference = SwerveState.fromTimedPose(desiredState);
            m_log_reference.log(() -> reference);
            FieldRelativeVelocity fieldRelativeTarget = m_controller.calculate(m_swerve.getState(), reference);
            m_swerve.driveInFieldCoords(fieldRelativeTarget);
        } else {
            // look one loop ahead by *previewing* the next point
            Optional<TrajectorySamplePoint> optSamplePoint = m_iter.preview(TimedRobot100.LOOP_PERIOD_S);
            if (optSamplePoint.isEmpty()) {
                Util.warn("broken trajectory, cancelling!");
                cancel(); // this should not happen
                return;
            }
            TrajectorySamplePoint samplePoint = optSamplePoint.get();
            TimedPose desiredState = samplePoint.state();

            SwerveState reference = SwerveState.fromTimedPose(desiredState);
            m_log_reference.log(() -> reference);
            FieldRelativeVelocity fieldRelativeTarget = m_controller.calculate(m_swerve.getState(), reference);
            m_aligned = m_swerve.steerAtRest(fieldRelativeTarget);
        }
    }

    @Override
    public boolean isFinished() {
        return done;
    }

    @Override
    public void end(boolean interrupted) {
        m_swerve.stop();
        m_viz.clear();
    }
}
