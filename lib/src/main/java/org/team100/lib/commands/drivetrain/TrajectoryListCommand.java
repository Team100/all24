package org.team100.lib.commands.drivetrain;

import java.util.Iterator;
import java.util.List;
import java.util.Optional;
import java.util.function.Function;

import org.team100.lib.commands.Command100;
import org.team100.lib.controller.DriveMotionController;
import org.team100.lib.controller.HolonomicFieldRelativeController;
import org.team100.lib.logging.SupplierLogger;
import org.team100.lib.motion.drivetrain.SwerveDriveSubsystem;
import org.team100.lib.motion.drivetrain.SwerveState;
import org.team100.lib.motion.drivetrain.kinodynamics.FieldRelativeVelocity;
import org.team100.lib.telemetry.Telemetry.Level;
import org.team100.lib.timing.TimedPose;
import org.team100.lib.trajectory.Trajectory100;
import org.team100.lib.trajectory.TrajectorySamplePoint;
import org.team100.lib.trajectory.TrajectoryTimeIterator;
import org.team100.lib.trajectory.TrajectoryTimeSampler;
import org.team100.lib.util.Util;
import org.team100.lib.visualization.TrajectoryVisualization;

import edu.wpi.first.math.geometry.Pose2d;

/**
 * Follow a list of trajectories.
 * 
 * The list can be relative to the current pose.
 * 
 * This just holds the starting rotation. If you want a holonomic trajectory
 * follower, try the {@link DriveMotionController} classes.
 */
public class TrajectoryListCommand extends Command100 {
    private final SwerveDriveSubsystem m_swerve;
    private final HolonomicFieldRelativeController m_controller;
    private final Function<Pose2d, List<Trajectory100>> m_trajectories;
    private final TrajectoryVisualization m_viz;

    private Iterator<Trajectory100> m_trajectoryIter;
    private TrajectoryTimeIterator m_iter;
    private boolean m_aligned;
    private boolean done;

    public TrajectoryListCommand(
            SupplierLogger parent,
            SwerveDriveSubsystem swerve,
            HolonomicFieldRelativeController controller,
            Function<Pose2d, List<Trajectory100>> trajectories,
            TrajectoryVisualization viz) {
        super(parent);
        m_swerve = swerve;
        m_controller = controller;
        m_trajectories = trajectories;
        m_viz = viz;
        addRequirements(m_swerve);
    }

    @Override
    public void initialize100() {
        m_controller.reset();
        Pose2d currentPose = m_swerve.getState().pose();
        m_trajectoryIter = m_trajectories.apply(currentPose).iterator();
        m_iter = null;
        done = false;
        m_aligned = false;
    }

    @Override
    public void execute100(double dt) {
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
            Optional<TrajectorySamplePoint> optSamplePoint = m_iter.advance(dt);
            if (optSamplePoint.isEmpty()) {
                Util.warn("broken trajectory, cancelling!");
                cancel(); // this should not happen
                return;
            }
            TrajectorySamplePoint samplePoint = optSamplePoint.get();
            TimedPose desiredState = samplePoint.state();

            Pose2d currentPose = m_swerve.getState().pose();
            SwerveState reference = SwerveState.fromTimedPose(desiredState);
            m_logger.logSwerveState(Level.TRACE, "reference", () -> reference);
            FieldRelativeVelocity fieldRelativeTarget = m_controller.calculate(currentPose, reference);
            m_swerve.driveInFieldCoords(fieldRelativeTarget, dt);
        } else {
            // look one loop ahead by *previewing* the next point
            Optional<TrajectorySamplePoint> optSamplePoint = m_iter.preview(dt);
            if (optSamplePoint.isEmpty()) {
                Util.warn("broken trajectory, cancelling!");
                cancel(); // this should not happen
                return;
            }
            TrajectorySamplePoint samplePoint = optSamplePoint.get();
            TimedPose desiredState = samplePoint.state();

            Pose2d currentPose = m_swerve.getState().pose();
            SwerveState reference = SwerveState.fromTimedPose(desiredState);
            m_logger.logSwerveState(Level.TRACE, "reference", () -> reference);
            FieldRelativeVelocity fieldRelativeTarget = m_controller.calculate(currentPose, reference);
            m_aligned = m_swerve.steerAtRest(fieldRelativeTarget, dt);
        }
    }

    @Override
    public boolean isFinished() {
        return done;
    }

    @Override
    public void end100(boolean interrupted) {
        m_swerve.stop();
        m_viz.clear();
    }
}
